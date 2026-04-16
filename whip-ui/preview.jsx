
import { useState, useEffect, useRef, useCallback } from "react";

// ── Simulator (pure JS, no server needed) ─────────────────────────────────
const PULSES_PER_REV = 400, GEAR_RATIO = 5.0, RAMP_STEPS = 150;

function calcMoveTime(p0, p1, speed, brk, soft) {
  const dist = Math.abs(p1 - p0); if (!dist) return 0;
  const steps = (dist * GEAR_RATIO / 360) * PULSES_PER_REV;
  let t = steps / Math.max(speed, 1);
  if (soft && steps > RAMP_STEPS) t += 0.15;
  if (brk  && steps > RAMP_STEPS) t += 0.15;
  return t;
}

function simulate(steps) {
  const events = [];
  let t = 0, pos = 0, angle = 90, speed = 2000, brk = true, soft = true;
  for (const step of steps) {
    const [cmd, val = null] = step;
    if (cmd === "left") {
      const p1 = pos + angle, dt = calcMoveTime(pos, p1, speed, brk, soft);
      events.push({ type:"move", t0:t, t1:t+dt, p0:pos, p1, speed, braking:brk, softstart:soft });
      pos = p1; t += dt;
    } else if (cmd === "right") {
      const p1 = pos - angle, dt = calcMoveTime(pos, p1, speed, brk, soft);
      events.push({ type:"move", t0:t, t1:t+dt, p0:pos, p1, speed, braking:brk, softstart:soft });
      pos = p1; t += dt;
    } else if (cmd === "neutral") {
      const p1 = Math.round(pos / 360) * 360, dt = calcMoveTime(pos, p1, speed, brk, soft);
      events.push({ type:"move", t0:t, t1:t+dt, p0:pos, p1, speed, braking:brk, softstart:soft });
      pos = p1; t += dt;
    } else if (cmd === "cw" || cmd === "ccw") {
      const dt = calcMoveTime(0, 360, speed, false, false);
      events.push({ type:"continuous", t0:t, t1:t+dt, dir: cmd==="cw"?1:-1, speed, pos });
      t += dt;
    } else if (cmd === "wait") {
      const ms = val ?? 500;
      events.push({ type:"wait", t0:t, t1:t+ms/1000, pos }); t += ms/1000;
    } else if (cmd === "wait_pend") {
      events.push({ type:"wait_pend", t0:t, t1:t+0.1, pos }); t += 0.1;
    } else if (cmd === "angle")       angle = val ?? 90;
    else if (cmd === "speed")         speed = val ?? 2000;
    else if (cmd === "brakeon")       brk = true;
    else if (cmd === "brakeoff")      brk = false;
    else if (cmd === "softstarton")   soft = true;
    else if (cmd === "softstartoff")  soft = false;
  }
  // tangle risk: direction reversal with <300ms gap
  for (let i = 1; i < events.length; i++) {
    const e = events[i], p = events[i-1];
    e.tangleRisk = false;
    if (e.type==="move" && p.type==="move")
      if (Math.sign(p.p1-p.p0) !== Math.sign(e.p1-e.p0) && (e.t0-p.t1) < 0.3)
        e.tangleRisk = true;
  }
  return { events, duration: t, finalPos: pos };
}

function getPos(timeline, loopT) {
  if (!timeline) return 0;
  let pos = 0;
  for (const e of timeline.events) {
    if (e.type === "move") {
      if (loopT >= e.t0 && loopT <= e.t1) {
        const p = (loopT - e.t0) / Math.max(e.t1 - e.t0, 0.001);
        const ease = p < 0.5 ? 2*p*p : 1 - Math.pow(-2*p+2,2)/2;
        return e.p0 + (e.p1 - e.p0) * ease;
      } else if (loopT > e.t1) pos = e.p1;
    } else if ((e.type==="wait"||e.type==="wait_pend") && loopT>=e.t0 && loopT<=e.t1) {
      return e.pos ?? pos;
    }
  }
  return pos;
}

// ── Arm Canvas ────────────────────────────────────────────────────────────
function ArmView({ angleDeg, chainHist }) {
  const ref = useRef();
  useEffect(() => {
    const canvas = ref.current; if (!canvas) return;
    const c = canvas.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    const W = canvas.width, H = canvas.height;
    const cx = W/2, cy = H/2, scale = Math.min(W,H) * 0.36;
    c.clearRect(0,0,W,H);

    // grid
    c.strokeStyle="#1a1a2a"; c.lineWidth=1;
    [0.35,0.7,1.0,1.35].forEach(r => { c.beginPath(); c.arc(cx,cy,scale*r,0,Math.PI*2); c.stroke(); });
    c.beginPath(); c.moveTo(cx-W,cy); c.lineTo(cx+W,cy); c.moveTo(cx,cy-H); c.lineTo(cx,cy+H); c.stroke();

    const rad = angleDeg * Math.PI / 180;
    const tipX = cx + Math.sin(rad)*scale*0.72, tipY = cy - Math.cos(rad)*scale*0.72;
    const ctrX = cx - Math.sin(rad)*scale*0.28, ctrY = cy + Math.cos(rad)*scale*0.28;

    // chain ghost trail
    if (chainHist.length > 2) {
      c.beginPath();
      chainHist.forEach((a, i) => {
        const r2 = a * Math.PI/180;
        const ex = cx + Math.sin(r2)*scale*1.22, ey = cy - Math.cos(r2)*scale*1.22;
        i === 0 ? c.moveTo(ex,ey) : c.lineTo(ex,ey);
      });
      c.strokeStyle="rgba(252,92,124,0.12)"; c.lineWidth=4*dpr; c.stroke();
    }

    // arm shadow
    c.beginPath(); c.moveTo(ctrX,ctrY); c.lineTo(tipX,tipY);
    c.strokeStyle="#2a1a3a"; c.lineWidth=10*dpr; c.lineCap="round"; c.stroke();
    // arm
    c.beginPath(); c.moveTo(ctrX,ctrY); c.lineTo(tipX,tipY);
    c.strokeStyle="#7c5cfc"; c.lineWidth=3.5*dpr; c.stroke();
    // counterweight
    c.beginPath(); c.arc(ctrX,ctrY,7*dpr,0,Math.PI*2);
    c.fillStyle="#3a2a5a"; c.fill(); c.strokeStyle="#7c5cfc"; c.lineWidth=1.5*dpr; c.stroke();
    // chain lag
    const lagDeg = chainHist.length > 6 ? angleDeg + (chainHist[chainHist.length-6] - angleDeg)*0.55 : angleDeg;
    const lr = lagDeg * Math.PI/180;
    const cex = cx + Math.sin(lr)*scale*1.22, cey = cy - Math.cos(lr)*scale*1.22;
    c.beginPath(); c.moveTo(tipX,tipY);
    c.quadraticCurveTo(tipX+(cex-tipX)*0.25, tipY+(cey-tipY)*0.65, cex, cey);
    c.strokeStyle="#fc5c7c"; c.lineWidth=2.5*dpr;
    c.setLineDash([4*dpr,3*dpr]); c.stroke(); c.setLineDash([]);
    // tassel
    c.beginPath(); c.arc(cex,cey,5*dpr,0,Math.PI*2); c.fillStyle="#fc5c7c"; c.fill();
    // hub
    c.beginPath(); c.arc(cx,cy,9*dpr,0,Math.PI*2);
    c.fillStyle="#1a1a2a"; c.fill(); c.strokeStyle="#5a5a9a"; c.lineWidth=2*dpr; c.stroke();
    // label
    c.fillStyle="#7c5cfc"; c.font=`${Math.round(12*dpr)}px monospace`;
    c.fillText(`${angleDeg.toFixed(1)}°`, 10*dpr, 18*dpr);
  }, [angleDeg, chainHist]);

  return (
    <canvas ref={ref} width={400} height={300}
      style={{width:"100%",height:"100%",borderRadius:8,background:"#0e0e1a",border:"1px solid #1e1e32",display:"block"}}/>
  );
}

// ── Timeline Canvas ────────────────────────────────────────────────────────
function TimelineView({ timeline, markerT }) {
  const ref = useRef();
  useEffect(() => {
    const canvas = ref.current; if (!canvas || !timeline) return;
    const c = canvas.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    const W = canvas.width, H = canvas.height;
    c.clearRect(0,0,W,H);
    const PAD=34*dpr, PW=W-PAD*2, PH=H-22*dpr, dur=timeline.duration||1;

    let minA=-20, maxA=20;
    timeline.events.forEach(e => { if(e.type==="move"){minA=Math.min(minA,e.p0,e.p1);maxA=Math.max(maxA,e.p0,e.p1);} });
    const pad=(maxA-minA)*0.12; minA-=pad; maxA+=pad;
    const aR=maxA-minA;
    const tx=t=>PAD+(t/dur)*PW, ay=a=>PH*0.05+PH*0.9*(1-(a-minA)/aR);

    c.strokeStyle="#2a2a42"; c.lineWidth=1;
    c.beginPath(); c.moveTo(PAD,ay(0)); c.lineTo(PAD+PW,ay(0)); c.stroke();

    timeline.events.forEach(e => {
      if (e.type==="move") {
        const col=e.tangleRisk?"#fc5c7c":(!e.braking&&!e.softstart)?"#fc9c4c":"#7c5cfc";
        if (e.tangleRisk) { c.fillStyle="rgba(252,92,124,0.07)"; c.fillRect(tx(e.t0),0,tx(e.t1)-tx(e.t0),H); }
        c.strokeStyle=col; c.lineWidth=e.tangleRisk?3:2;
        c.beginPath(); c.moveTo(tx(e.t0),ay(e.p0)); c.lineTo(tx(e.t1),ay(e.p1)); c.stroke();
      } else if (e.type==="wait"||e.type==="wait_pend") {
        c.strokeStyle="#3a3a5a"; c.lineWidth=1.5;
        c.setLineDash([3*dpr,3*dpr]);
        c.beginPath(); c.moveTo(tx(e.t0),ay(e.pos)); c.lineTo(tx(e.t1),ay(e.pos)); c.stroke();
        c.setLineDash([]);
      }
    });

    const step=dur<=10?1:dur<=30?5:10;
    c.fillStyle="#5a5a7a"; c.font=`${8*dpr}px monospace`;
    for (let tt=0; tt<=dur; tt+=step) {
      const x=tx(tt);
      c.strokeStyle="#1e1e2e"; c.lineWidth=1; c.beginPath(); c.moveTo(x,0); c.lineTo(x,PH); c.stroke();
      c.fillText(`${tt.toFixed(0)}s`, x-5*dpr, H-4*dpr);
    }
    if (markerT>0) {
      c.strokeStyle="rgba(232,232,248,0.35)"; c.lineWidth=1.5;
      c.beginPath(); c.moveTo(tx(Math.min(markerT,dur)),0); c.lineTo(tx(Math.min(markerT,dur)),H); c.stroke();
    }
    [["#7c5cfc","smooth"],["#fc9c4c","sharp"],["#fc5c7c","⚠ tangle"]].forEach(([col,lbl],i)=>{
      c.fillStyle=col; c.fillRect(PAD+i*68*dpr,H-12*dpr,6*dpr,6*dpr);
      c.fillStyle="#5a5a7a"; c.fillText(lbl,PAD+i*68*dpr+9*dpr,H-5*dpr);
    });
  }, [timeline, markerT]);

  return (
    <canvas ref={ref} width={600} height={120}
      style={{width:"100%",height:"100%",borderRadius:6,background:"#0e0e1a",border:"1px solid #1e1e32",display:"block"}}/>
  );
}

// ── Styles ─────────────────────────────────────────────────────────────────
const S = {
  app: { display:"grid", gridTemplateColumns:"290px 1fr", gridTemplateRows:"44px 1fr",
         height:"100vh", overflow:"hidden", background:"#080810", color:"#c8c8e0",
         fontFamily:"'Space Grotesk',sans-serif", fontSize:13 },
  topbar: { gridColumn:"1/-1", display:"flex", alignItems:"center", gap:8, padding:"0 14px",
            background:"#0e0e1a", borderBottom:"1px solid #1e1e32" },
  h1: { fontFamily:"monospace", fontSize:11, letterSpacing:"0.15em", textTransform:"uppercase",
        color:"#5a5a7a", flex:1 },
  left: { display:"flex", flexDirection:"column", borderRight:"1px solid #1e1e32", overflow:"hidden" },
  tabs: { display:"flex", borderBottom:"1px solid #1e1e32" },
  tab: (active) => ({ flex:1, padding:"10px 4px", textAlign:"center", fontFamily:"monospace",
    fontSize:10, letterSpacing:"0.08em", textTransform:"uppercase", cursor:"pointer",
    color: active?"#e8e8f8":"#5a5a7a",
    borderBottom: active?"2px solid #7c5cfc":"2px solid transparent", marginBottom:-1 }),
  pane: { display:"flex", flex:1, overflowY:"auto", padding:11, flexDirection:"column", gap:9 },
  lbl: { fontFamily:"monospace", fontSize:9, letterSpacing:"0.15em", textTransform:"uppercase",
         color:"#5a5a7a", marginBottom:4 },
  btn: (variant="default") => {
    const colors = {
      default: { bg:"#0e0e1a", border:"#1e1e32", color:"#c8c8e0" },
      accent:  { bg:"#1a0d3a", border:"#7c5cfc", color:"#7c5cfc" },
      green:   { bg:"#0d2a1a", border:"#4cfc9c", color:"#4cfc9c" },
      red:     { bg:"#2a0d0d", border:"#fc5c7c", color:"#fc5c7c" },
      warn:    { bg:"#2a1a0d", border:"#fc9c4c", color:"#fc9c4c" },
    };
    const { bg, border, color } = colors[variant] || colors.default;
    return { flex:1, padding:"7px 5px", border:`1px solid ${border}`, borderRadius:6,
             background:bg, color, fontFamily:"monospace", fontSize:11, cursor:"pointer",
             textAlign:"center", userSelect:"none" };
  },
  btnSm: (variant="default") => ({ ...S.btn(variant), padding:"4px 6px", fontSize:10 }),
  sliderBlock: { display:"flex", flexDirection:"column", gap:3 },
  sliderRow: { display:"flex", alignItems:"center", gap:8 },
  sliderVal: { fontFamily:"monospace", fontSize:12, color:"#e8e8f8", minWidth:52, textAlign:"right" },
  notice: { background:"#1a1a0a", border:"1px solid #3a3a0a", borderRadius:6,
            padding:"8px 10px", fontFamily:"monospace", fontSize:10, color:"#fc9c4c", lineHeight:1.7 },
  seqList: { background:"#080810", border:"1px solid #1e1e32", borderRadius:5,
             overflowY:"auto", maxHeight:200, minHeight:32 },
  sitem: (sel, risk) => ({ display:"flex", alignItems:"center", gap:5, padding:"5px 7px",
    borderBottom:"1px solid #1e1e32", fontFamily:"monospace", fontSize:11, cursor:"pointer",
    background: sel?"#1a0d3a":"transparent",
    borderLeft: risk?"3px solid #fc5c7c":"3px solid transparent" }),
  right: { display:"flex", flexDirection:"column", overflow:"hidden" },
  vizBar: { display:"flex", alignItems:"center", gap:7, padding:"7px 12px",
            borderBottom:"1px solid #1e1e32", background:"#0e0e1a", flexShrink:0 },
  vizWrap: { flex:1, display:"grid", gridTemplateRows:"1fr 120px", overflow:"hidden", padding:8, gap:7 },
};

const Toggle = ({ on, onClick }) => (
  <div onClick={onClick} style={{
    position:"relative", width:30, height:17, borderRadius:9, cursor:"pointer",
    background: on?"#7c5cfc":"#2a2a42", transition:"background 0.2s",
  }}>
    <div style={{
      position:"absolute", width:11, height:11, borderRadius:"50%", background:"#080810",
      top:3, left: on?16:3, transition:"left 0.2s",
    }}/>
  </div>
);

const NO_VAL = new Set(["left","right","neutral","cw","ccw","wait_pend",
  "brakeon","brakeoff","softstarton","softstartoff","on","off","calibrate"]);

const CMDS = [
  { group:"Motion",     opts:["left","right","neutral","cw","ccw"] },
  { group:"Timing",     opts:["wait","wait_pend"] },
  { group:"Parameters", opts:["angle","speed","delay"] },
  { group:"Flags",      opts:["brakeon","brakeoff","softstarton","softstartoff","on","off","calibrate"] },
];

const DEFAULTS = { wait:500, angle:90, speed:2000, delay:500 };

// ── Main component ─────────────────────────────────────────────────────────
export default function App() {
  const [tab, setTab] = useState("compose");
  const [sequence, setSequence] = useState([]);
  const [selectedIdx, setSelectedIdx] = useState(-1);
  const [timeline, setTimeline] = useState(null);
  const [stepCmd, setStepCmd] = useState("left");
  const [stepVal, setStepVal] = useState(500);
  const [seqName, setSeqName] = useState("");
  const [currentSpeed, setCurrentSpeed] = useState(2000);
  const [currentAngle, setCurrentAngle] = useState(90);
  const [savedSeqs, setSavedSeqs] = useState({});

  // animation
  const [animAngle, setAnimAngle] = useState(0);
  const [chainHist, setChainHist] = useState([]);
  const [markerT, setMarkerT] = useState(0);
  const [playing, setPlaying] = useState(false);
  const animRef = useRef(null);
  const animStartRef = useRef(null);
  const timelineRef = useRef(null);

  // ── Animation loop ────────────────────────────────────────────────────
  const startAnim = useCallback((tl) => {
    if (animRef.current) cancelAnimationFrame(animRef.current);
    timelineRef.current = tl;
    animStartRef.current = null;
    setChainHist([]);
    setPlaying(true);

    const frame = (ts) => {
      if (!animStartRef.current) animStartRef.current = ts;
      const tl2 = timelineRef.current;
      if (!tl2 || !tl2.duration) return;
      const loopT = ((ts - animStartRef.current) / 1000) % (tl2.duration + 0.8);
      const pos = getPos(tl2, loopT);
      setAnimAngle(pos);
      setMarkerT(Math.min(loopT, tl2.duration));
      setChainHist(h => { const n = [...h, pos]; return n.length > 50 ? n.slice(-50) : n; });
      animRef.current = requestAnimationFrame(frame);
    };
    animRef.current = requestAnimationFrame(frame);
  }, []);

  const stopAnim = useCallback(() => {
    if (animRef.current) { cancelAnimationFrame(animRef.current); animRef.current = null; }
    setPlaying(false);
  }, []);

  useEffect(() => () => stopAnim(), [stopAnim]);

  // ── Preview ──────────────────────────────────────────────────────────
  const preview = () => {
    if (!sequence.length) return;
    const tl = simulate(sequence);
    setTimeline(tl);
    timelineRef.current = tl;
    startAnim(tl);
  };

  // ── Sequence ops ─────────────────────────────────────────────────────
  const addStep = () => {
    const step = NO_VAL.has(stepCmd) ? [stepCmd] : [stepCmd, +stepVal];
    setSequence(s => [...s, step]);
  };

  const removeStep = (i) => setSequence(s => s.filter((_,j) => j !== i));

  const moveUp = () => {
    if (selectedIdx < 1) return;
    const s = [...sequence];
    [s[selectedIdx-1], s[selectedIdx]] = [s[selectedIdx], s[selectedIdx-1]];
    setSequence(s); setSelectedIdx(selectedIdx - 1);
  };

  const moveDn = () => {
    if (selectedIdx < 0 || selectedIdx >= sequence.length-1) return;
    const s = [...sequence];
    [s[selectedIdx], s[selectedIdx+1]] = [s[selectedIdx+1], s[selectedIdx]];
    setSequence(s); setSelectedIdx(selectedIdx + 1);
  };

  const dupStep = () => {
    if (selectedIdx < 0) return;
    const s = [...sequence];
    s.splice(selectedIdx+1, 0, [...s[selectedIdx]]);
    setSequence(s);
  };

  const clearSeq = () => { if (sequence.length && !window.confirm("Clear sequence?")) return; setSequence([]); setSelectedIdx(-1); setTimeline(null); stopAnim(); };

  const addPattern = (name) => {
    const a = currentAngle, s = currentSpeed;
    const P = {
      pendulum: [["on"],["softstarton"],["brakeon"],["speed",s],["angle",a],
        ...Array(4).fill(0).flatMap(()=>[["left"],["wait",600],["right"],["wait",600]]),
        ["neutral"],["off"]],
      buildup: [["on"],["speed",s],
        ...[30,60,90,120,180].flatMap(ag=>[["angle",ag],["left"],["wait",700],["right"],["wait",700]]),
        ["neutral"],["off"]],
      spin: [["on"],["speed",s],["cw"],["wait",3000],["speed",Math.round(s*0.4)],["ccw"],["wait",3000],["neutral"],["off"]],
      wave: [["on"],["softstarton"],["brakeon"],
        ["speed",Math.round(s*0.3)],["angle",30],
        ...Array(3).fill(0).flatMap(()=>[["left"],["wait",250],["right"],["wait",250]]),
        ["speed",s],["angle",a],
        ...Array(3).fill(0).flatMap(()=>[["left"],["wait",500],["right"],["wait",500]]),
        ["neutral"],["off"]],
    };
    setSequence(prev => [...prev, ...(P[name]||[])]);
  };

  const saveSeq = () => {
    const name = (seqName.trim() || "untitled").replace(/[/\\]/g,"-");
    setSavedSeqs(s => ({ ...s, [name]: { name, steps: sequence } }));
  };

  const loadSeq = (name) => {
    const d = savedSeqs[name]; if (!d) return;
    setSequence(d.steps); setSeqName(name);
    setSelectedIdx(-1); setTimeline(null); stopAnim();
    setTab("compose");
  };

  const deleteSeq = (name) => {
    if (!window.confirm(`Delete "${name}"?`)) return;
    setSavedSeqs(s => { const n={...s}; delete n[name]; return n; });
  };

  const exportJSON = () => {
    const name = seqName.trim() || "sequence";
    const blob = new Blob([JSON.stringify({name,steps:sequence},null,2)],{type:"application/json"});
    const a = document.createElement("a"); a.href=URL.createObjectURL(blob); a.download=`${name}.json`; a.click();
  };

  const importJSON = (e) => {
    const file = e.target.files[0]; if (!file) return;
    const r = new FileReader();
    r.onload = ev => {
      try { const d=JSON.parse(ev.target.result); setSequence(d.steps||[]); setSeqName(d.name||""); }
      catch { alert("Invalid JSON"); }
    };
    r.readAsText(file);
  };

  // risks
  const risks = timeline ? timeline.events.filter(e => e.tangleRisk).length : 0;

  return (
    <div style={S.app}>
      {/* topbar */}
      <div style={S.topbar}>
        <h1 style={S.h1}>On Acquiescence</h1>
        <span style={{fontFamily:"monospace",fontSize:10,padding:"3px 9px",borderRadius:100,
          background:"#1a1a0a",color:"#fc9c4c"}}>OFFLINE MODE</span>
        <span style={{fontFamily:"monospace",fontSize:10,color:"#5a5a7a"}}>
          enter Pi4 IP in the downloaded file to go live
        </span>
      </div>

      {/* left */}
      <div style={S.left}>
        <div style={S.tabs}>
          {["compose","library"].map(t => (
            <div key={t} style={S.tab(tab===t)} onClick={()=>setTab(t)}>{t}</div>
          ))}
        </div>

        {/* COMPOSE */}
        {tab === "compose" && (
          <div style={S.pane}>
            <div style={S.notice}>
              ◈ OFFLINE — build &amp; visualise freely.<br/>
              The downloaded file connects to your Pi4 for live control.
            </div>

            <div>
              <div style={S.lbl}>Quick patterns</div>
              <div style={{display:"flex",gap:5}}>
                {["pendulum","buildup","spin","wave"].map(p => (
                  <div key={p} style={S.btnSm()} onClick={()=>addPattern(p)}>{p}</div>
                ))}
              </div>
            </div>

            <div>
              <div style={S.lbl}>Params for patterns</div>
              <div style={{display:"flex",gap:8,alignItems:"center"}}>
                <span style={{fontFamily:"monospace",fontSize:10,color:"#5a5a7a"}}>speed</span>
                <input type="number" value={currentSpeed} onChange={e=>setCurrentSpeed(+e.target.value)}
                  style={{width:70,background:"#0e0e1a",border:"1px solid #1e1e32",color:"#c8c8e0",
                    borderRadius:4,padding:"3px 6px",fontFamily:"monospace",fontSize:11}}/>
                <span style={{fontFamily:"monospace",fontSize:10,color:"#5a5a7a"}}>angle</span>
                <input type="number" value={currentAngle} onChange={e=>setCurrentAngle(+e.target.value)}
                  style={{width:55,background:"#0e0e1a",border:"1px solid #1e1e32",color:"#c8c8e0",
                    borderRadius:4,padding:"3px 6px",fontFamily:"monospace",fontSize:11}}/>
              </div>
            </div>

            <div>
              <div style={S.lbl}>Add step</div>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:5}}>
                <select value={stepCmd} onChange={e=>{setStepCmd(e.target.value);setStepVal(DEFAULTS[e.target.value]??500);}}
                  style={{background:"#0e0e1a",border:"1px solid #1e1e32",color:"#c8c8e0",
                    borderRadius:5,padding:"5px 7px",fontFamily:"monospace",fontSize:11}}>
                  {CMDS.map(({group,opts})=>(
                    <optgroup key={group} label={group}>
                      {opts.map(o=><option key={o} value={o}>{o}</option>)}
                    </optgroup>
                  ))}
                </select>
                <input type="number" value={stepVal} onChange={e=>setStepVal(+e.target.value)}
                  disabled={NO_VAL.has(stepCmd)}
                  style={{opacity:NO_VAL.has(stepCmd)?0.3:1,background:"#0e0e1a",
                    border:"1px solid #1e1e32",color:"#c8c8e0",borderRadius:5,
                    padding:"5px 7px",fontFamily:"monospace",fontSize:11}}/>
              </div>
              <div style={{marginTop:5}}>
                <div style={{...S.btn("green"),flex:"none",width:"100%"}} onClick={addStep}>+ ADD STEP</div>
              </div>
            </div>

            <div>
              <div style={{...S.lbl,display:"flex",justifyContent:"space-between"}}>
                <span>Sequence</span>
                <span style={{color:"#5a5a7a",fontSize:9}}>
                  {sequence.length} steps{timeline?` · ${timeline.duration.toFixed(1)}s`:""}
                  {risks>0?` · ⚠${risks}`:""}
                </span>
              </div>
              <div style={S.seqList}>
                {sequence.length === 0 && (
                  <div style={{fontFamily:"monospace",fontSize:10,color:"#5a5a7a",padding:8}}>
                    No steps yet. Add steps above or load a quick pattern.
                  </div>
                )}
                {sequence.map(([c,v],i) => (
                  <div key={i} style={S.sitem(i===selectedIdx, timeline?.events[i]?.tangleRisk)}
                    onClick={()=>setSelectedIdx(i)}>
                    <span style={{color:"#5a5a7a",minWidth:18,fontSize:10}}>{i+1}</span>
                    <span style={{color:"#7c5cfc",flex:1}}>{c}</span>
                    <span style={{color:"#fc9c4c"}}>{v!==undefined?v:""}</span>
                    <span style={{color:"#5a5a7a",padding:"1px 4px",cursor:"pointer"}}
                      onClick={e=>{e.stopPropagation();removeStep(i);}}>✕</span>
                  </div>
                ))}
              </div>
              <div style={{display:"flex",gap:5,marginTop:5}}>
                <div style={S.btnSm("red")} onClick={clearSeq}>CLEAR</div>
                <div style={S.btnSm()} onClick={moveUp}>↑</div>
                <div style={S.btnSm()} onClick={moveDn}>↓</div>
                <div style={S.btnSm()} onClick={dupStep}>DUP</div>
              </div>
            </div>

            <div>
              <div style={S.lbl}>Preview &amp; export</div>
              <div style={{display:"flex",gap:5}}>
                <div style={S.btnSm("accent")} onClick={preview}>▶ PREVIEW</div>
                <div style={S.btnSm()} onClick={exportJSON}>↓ JSON</div>
                <label style={S.btnSm()}>
                  ↑ IMPORT
                  <input type="file" accept=".json" style={{display:"none"}} onChange={importJSON}/>
                </label>
              </div>
              <div style={{display:"flex",gap:5,marginTop:5}}>
                <input value={seqName} onChange={e=>setSeqName(e.target.value)}
                  placeholder="sequence name"
                  style={{flex:1,background:"#0e0e1a",border:"1px solid #1e1e32",color:"#c8c8e0",
                    borderRadius:5,padding:"5px 7px",fontFamily:"monospace",fontSize:11}}/>
                <div style={{...S.btnSm("green"),flex:"none",whiteSpace:"nowrap"}} onClick={saveSeq}>SAVE</div>
              </div>
            </div>
          </div>
        )}

        {/* LIBRARY */}
        {tab === "library" && (
          <div style={S.pane}>
            <div style={S.lbl}>Saved sequences</div>
            {Object.keys(savedSeqs).length === 0 && (
              <div style={{fontFamily:"monospace",fontSize:10,color:"#5a5a7a",padding:"6px 0"}}>
                No saved sequences yet. Build one in COMPOSE and save it.
              </div>
            )}
            {Object.keys(savedSeqs).sort().map(name => (
              <div key={name} style={{display:"flex",alignItems:"center",gap:5,padding:"6px 8px",
                border:"1px solid #1e1e32",borderRadius:5,background:"#0e0e1a"}}>
                <span style={{flex:1,fontFamily:"monospace",fontSize:11}}>{name}</span>
                <div style={S.btnSm()} onClick={()=>loadSeq(name)}>LOAD</div>
                <div style={S.btnSm("red")} onClick={()=>deleteSeq(name)}>✕</div>
              </div>
            ))}
            <div style={{...S.notice,marginTop:8}}>
              RUN buttons appear here when connected to Pi4 in the downloaded file.
            </div>
          </div>
        )}
      </div>

      {/* right: visualizer */}
      <div style={S.right}>
        <div style={S.vizBar}>
          <span style={{...S.lbl,margin:0}}>VISUALIZER</span>
          <div style={S.btnSm("accent")} onClick={preview}>▶ PREVIEW</div>
          <div style={S.btnSm()} onClick={stopAnim}>■ STOP</div>
          <div style={S.btnSm()} onClick={()=>{setChainHist([]);setAnimAngle(0);}}>⌖ RESET</div>
          <span style={{fontFamily:"monospace",fontSize:10,color:"#5a5a7a",flex:1,textAlign:"right"}}>
            {timeline
              ? `${sequence.length} steps · ${timeline.duration.toFixed(1)}s${risks>0?` · ⚠ ${risks} tangle risk${risks>1?"s":""}`:""}`
              : "Build a sequence then hit ▶ PREVIEW"}
          </span>
        </div>
        <div style={S.vizWrap}>
          <ArmView angleDeg={animAngle} chainHist={chainHist}/>
          <TimelineView timeline={timeline} markerT={markerT}/>
        </div>
      </div>
    </div>
  );
}
