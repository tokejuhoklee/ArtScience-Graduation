import { useState, useEffect, useRef, useCallback } from "react";

// ─── Theme ────────────────────────────────────────────────────────────────────
const BG   = "#07070e";
const SURF = "#0e0e1a";
const BDR  = "#181828";
const MUT  = "#4a4a6a";

// ─── Physics ─────────────────────────────────────────────────────────────────
const PPR = 400, GR = 5;
function moveTime(deg, spd, brk, soft) {
  const steps = (Math.abs(deg) * GR / 360) * PPR;
  if (!steps) return 0;
  let t = steps / Math.max(spd, 1);
  if (soft && steps > 150) t += 0.15;
  if (brk  && steps > 150) t += 0.15;
  return t;
}

// ─── Block → events ──────────────────────────────────────────────────────────
function expandBlock(blk, G, pos0) {
  const spd  = blk.speed  ?? G.speed;
  const ang  = (blk.angle  ?? G.angle)  * (G.angleMult ?? 1);
  const rest = (blk.restMs ?? G.restMs) * (G.restMult  ?? 1);
  const brk  = blk.brk  ?? G.brk;
  const soft = blk.soft ?? G.soft;
  const id   = blk.id;
  const ev   = [];
  let t = 0, pos = pos0;

  const wait = (ms) => {
    if (ms <= 0) return;
    ev.push({ type:"wait", t0:t, t1:t+ms/1000, pos, id });
    t += ms/1000;
  };

  const inv = ((G.invert ?? false) !== (blk.invert ?? false)) ? -1 : 1;

  // Helper for moves at a specific speed (wave/buildup vary speed per phase)
  const moveAt = (to, s) => {
    const dt = moveTime(Math.abs(to - pos), s, brk, soft);
    if (dt > 0) ev.push({ type:"move", t0:t, t1:t+dt, p0:pos, p1:to, brk, soft, id });
    pos = to; t += dt;
  };

  // Per-rep speed: lerp from spd → blk.speedEnd across reps (if speedEnd is set)
  const spdAt = i => {
    if (blk.speedEnd == null || blk.reps <= 1) return spd;
    return spd + (blk.speedEnd - spd) * i / (blk.reps - 1);
  };

  if (blk.type === "pendulum") {
    if (blk.oneSided) {
      const driftRatio = blk.driftRatio ?? 0.5;
      for (let i = 0; i < blk.reps; i++) {
        const si = spdAt(i);
        moveAt(pos + ang * inv,                  si); wait(rest);
        moveAt(pos - ang * driftRatio * inv,     si); wait(rest);
      }
    } else {
      for (let i = 0; i < blk.reps; i++) {
        const si = spdAt(i);
        moveAt( inv * ang, si); wait(rest);
        moveAt(-inv * ang, si); wait(rest);
      }
    }
  } else if (blk.type === "wave") {
    // Relative oscillation (offset-based) so it works from any arm position.
    // Pattern: each half-swing accounts for where the previous one ended.
    // After rep[i]: arm is at pos0 - inv*ga[i]. Rep[i+1] goes pos0 ± ga[i+1].
    const sAng = Math.max(5, Math.round(ang * 0.25));
    const sRest = Math.round(rest * 0.3);
    const smallReps = Math.max(2, Math.round(blk.reps * 0.5));
    const waveSwing = (a, s, r, prevA) => {
      moveAt(pos + inv * (a + prevA), s); wait(r);
      moveAt(pos - inv * 2 * a,       s); wait(r);
    };
    if (blk.constant) {
      let prevA = 0;
      for (let i = 0; i < smallReps; i++) { waveSwing(sAng, spd, sRest, prevA); prevA = sAng; }
      for (let i = 0; i < blk.reps; i++) {
        const ga = Math.round(sAng + (ang-sAng)*(i+1)/blk.reps);
        waveSwing(ga, spd, rest, prevA); prevA = ga;
      }
    } else {
      const sSpd = Math.round(spd * 0.3);
      let prevA = 0;
      for (let i = 0; i < smallReps; i++) { waveSwing(sAng, sSpd, sRest, prevA); prevA = sAng; }
      for (let i = 0; i < blk.reps; i++) {
        const frac = (i+1)/blk.reps;
        const ga = Math.round(sAng + (ang-sAng)*frac);
        const gs = Math.round(sSpd + (spd-sSpd)*frac);
        waveSwing(ga, gs, rest, prevA); prevA = ga;
      }
    }

  } else if (blk.type === "spin") {
    const revs = blk.continuous ? 10 : (blk.revs ?? 2);
    const dirs = blk.dir === "both" ? [inv, -inv] : [inv * (blk.dir ?? 1)];
    for (const dir of dirs) {
      const deg = revs * 360;
      const dt  = moveTime(deg, spd, false, false);
      ev.push({ type:"spin", t0:t, t1:t+dt, p0:pos, p1:pos+dir*deg, dir, revs, id });
      pos += dir*deg; t += dt;
      wait(rest);
    }

  } else if (blk.type === "buildup") {
    // Relative oscillation — oscillates around current position with growing amplitude
    let prevA = 0;
    for (let i = 0; i < blk.reps; i++) {
      const frac = (i+1)/blk.reps;
      const a = Math.max(5, Math.round(ang * frac));
      const s = Math.round(spd * (0.35 + 0.65*frac));
      const r = Math.round(rest * (1.6 - 0.9*frac));
      moveAt(pos + inv * (a + prevA), s); wait(r);
      moveAt(pos - inv * 2 * a,       s); wait(r);
      prevA = a;
    }

  } else if (blk.type === "swing") {
    // Single directional move — transition block
    const dir = blk.dir ?? 1;
    moveAt(pos + inv * dir * ang, spd);
    wait(rest);

  } else if (blk.type === "pause") {
    wait(blk.ms ?? 1000);
  }

  return { ev, endPos: pos, dur: t };
}

// ─── Block → Pi4 step list compiler ─────────────────────────────────────────
// The Pi4 server.py SequenceEngine expects: [{steps:[["cmd",val],...]}, ...]
// Pico left/right use absolute positions: left=+ang, right=-ang (from zero).
// We track position to calculate correct travel distances for wait times.
function compileToSteps(blocks, G) {
  const out = [
    ["stop"],
    ["on"],
    ["loop_start"],  // server restarts from here on subsequent loop iterations
  ];

  for (const blk of blocks) {
    const spd  = blk.speed  ?? G.speed;
    const ang  = Math.round((blk.angle  ?? G.angle)  * (G.angleMult ?? 1));
    const rest = Math.round((blk.restMs ?? G.restMs) * (G.restMult  ?? 1));

    // inv=-1 when exactly one of (global invert, block invert) is true — XOR
    const inv = ((G.invert ?? false) !== (blk.invert ?? false)) ? -1 : 1;

    // Per-block braking/soft-start (falls back to global)
    const effBrk  = blk.brk  ?? G.brk;
    const effSoft = blk.soft ?? G.soft;
    out.push(effBrk  ? ["brakeon"]     : ["brakeoff"]);
    out.push(effSoft ? ["softstarton"] : ["softstartoff"]);

    if (blk.type === "pendulum") {
      const angSteps = Math.round((ang * GR / 360) * PPR);
      const hasRamp  = blk.speedEnd != null && blk.reps > 1;
      if (!hasRamp) out.push(["speed", spd]);

      const repSpd = i => hasRamp
        ? Math.round(spd + (blk.speedEnd - spd) * i / (blk.reps - 1))
        : spd;

      if (blk.oneSided) {
        const driftRatio = blk.driftRatio ?? 0.5;
        const returnSteps = Math.round(angSteps * driftRatio);
        for (let i = 0; i < blk.reps; i++) {
          if (hasRamp) out.push(["speed", repSpd(i)]);
          out.push(["offset",  inv * angSteps],    ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
          out.push(["offset", -inv * returnSteps], ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
        }
      } else {
        for (let i = 0; i < blk.reps; i++) {
          if (hasRamp) out.push(["speed", repSpd(i)]);
          out.push(["move",  inv * angSteps], ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
          out.push(["move", -inv * angSteps], ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
        }
      }
    } else if (blk.type === "wave") {
      // Offset-based (relative) so it works from any arm position.
      // prevSteps tracks where the arm ended last half-swing so the next
      // half-swing can reach the correct peak: offset +(next+prev) / -(2*next).
      const sAng      = Math.max(5, Math.round(ang * 0.25));
      const sRest     = Math.round(rest * 0.3);
      const smallReps = Math.max(2, Math.round(blk.reps * 0.5));
      const sAngSteps = Math.round((sAng * GR / 360) * PPR);
      const pushSwing = (steps, spd_, rest_) => {
        out.push(["offset",  inv * (steps + prevSteps)], ["wait_pend"]);
        if (rest_ > 0) out.push(["wait", rest_]);
        out.push(["offset", -inv * 2 * steps], ["wait_pend"]);
        if (rest_ > 0) out.push(["wait", rest_]);
        prevSteps = steps;
      };
      let prevSteps = 0;
      if (blk.constant) {
        out.push(["speed", spd]);
        for (let i = 0; i < smallReps; i++) pushSwing(sAngSteps, spd, sRest);
        for (let i = 0; i < blk.reps; i++) {
          const gaSteps = Math.round((Math.round(sAng + (ang-sAng)*(i+1)/blk.reps) * GR / 360) * PPR);
          pushSwing(gaSteps, spd, rest);
        }
      } else {
        const sSpd = Math.round(spd * 0.3);
        out.push(["speed", sSpd]);
        for (let i = 0; i < smallReps; i++) pushSwing(sAngSteps, sSpd, sRest);
        for (let i = 0; i < blk.reps; i++) {
          const frac    = (i+1) / blk.reps;
          const gaSteps = Math.round((Math.round(sAng + (ang-sAng)*frac) * GR / 360) * PPR);
          const gs      = Math.round(sSpd + (spd-sSpd)*frac);
          out.push(["speed", gs]);
          pushSwing(gaSteps, gs, rest);
        }
        out.push(["speed", spd]);
      }

    } else if (blk.type === "spin") {
      out.push(["speed", spd]);
      if (blk.continuous) {
        if (blk.dir === "both") {
          const halfSteps = Math.round((2 * 360 * GR / 360) * PPR);
          out.push(["offset",  inv * halfSteps], ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
          out.push(["offset", -inv * halfSteps], ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
        } else {
          // inv flips CW↔CCW
          const effDir = inv * (blk.dir ?? 1);
          out.push([effDir > 0 ? "cw" : "ccw"]);
        }
      } else {
        const revs = blk.revs ?? 2;
        const dirs = blk.dir === "both" ? [inv, -inv] : [inv * (blk.dir ?? 1)];
        for (const dir of dirs) {
          const totalSteps = Math.round((revs * 360 * GR / 360) * PPR);
          out.push(["offset", dir * totalSteps], ["wait_pend"]);
          if (rest > 0) out.push(["wait", rest]);
        }
      }

    } else if (blk.type === "buildup") {
      // Offset-based — oscillates around current position with growing amplitude
      let prevSteps = 0;
      for (let i = 0; i < blk.reps; i++) {
        const frac    = (i+1) / blk.reps;
        const a       = Math.max(5, Math.round(ang * frac));
        const s       = Math.round(spd * (0.35 + 0.65*frac));
        const r       = Math.round(rest * (1.6 - 0.9*frac));
        const aSteps  = Math.round((a * GR / 360) * PPR);
        out.push(["speed", s]);
        out.push(["offset",  inv * (aSteps + prevSteps)], ["wait_pend"]);
        if (r > 0) out.push(["wait", r]);
        out.push(["offset", -inv * 2 * aSteps], ["wait_pend"]);
        if (r > 0) out.push(["wait", r]);
        prevSteps = aSteps;
      }
      out.push(["speed", spd]);

    } else if (blk.type === "swing") {
      // Single directional offset move — use as a transition between blocks
      const angSteps = Math.round((ang * GR / 360) * PPR);
      const dir = blk.dir ?? 1;
      out.push(["speed", spd]);
      out.push(["offset", inv * dir * angSteps], ["wait_pend"]);
      if (rest > 0) out.push(["wait", rest]);

    } else if (blk.type === "pause") {
      out.push(["wait", blk.ms ?? 1000]);
    }
  }
  return out;
}

function simulate(blocks, G) {
  const allEv = [];
  let t = 0, pos = 0;
  for (const blk of blocks) {
    const { ev, endPos, dur } = expandBlock(blk, G, pos);
    ev.forEach(e => { allEv.push({...e, t0:e.t0+t, t1:e.t1+t}); });
    t += dur; pos = endPos;
  }
  // tangle risk
  for (let i=1; i<allEv.length; i++) {
    const e=allEv[i], p=allEv[i-1];
    e.risk = e.type==="move" && p.type==="move"
      && Math.sign(p.p1-p.p0) !== Math.sign(e.p1-e.p0)
      && (e.t0 - p.t1) < 0.25;
  }
  if (allEv.length) allEv[0].risk = false;
  return { ev: allEv, dur: t };
}

function posAt(ev, t) {
  let pos = 0;
  for (const e of ev) {
    if (e.type==="move"||e.type==="spin") {
      if (t >= e.t0 && t <= e.t1) {
        const p = (t-e.t0)/Math.max(e.t1-e.t0,0.001);
        const f = e.type==="spin" ? p : p<0.5 ? 2*p*p : 1-Math.pow(-2*p+2,2)/2;
        return e.p0 + (e.p1-e.p0)*f;
      }
      if (t > e.t1) pos = e.p1;
    } else if (e.type==="wait") {
      if (t >= e.t0 && t <= e.t1) return e.pos;
      if (t > e.t1) pos = e.pos;
    }
  }
  return pos;
}

// ─── Arm canvas ───────────────────────────────────────────────────────────────
function ArmCanvas({ deg, trail }) {
  const ref = useRef();
  useEffect(() => {
    const cv = ref.current; if (!cv) return;
    const dpr = window.devicePixelRatio || 1;
    const { width: rw, height: rh } = cv.getBoundingClientRect();
    cv.width = rw*dpr; cv.height = rh*dpr;
    const g = cv.getContext("2d");
    const W=cv.width, H=cv.height, cx=W/2, cy=H/2, sc=Math.min(W,H)*0.36;
    g.clearRect(0,0,W,H);

    // rings + crosshair
    g.strokeStyle="#161625"; g.lineWidth=1;
    [0.38,0.72,1.0,1.28].forEach(r=>{g.beginPath();g.arc(cx,cy,sc*r,0,Math.PI*2);g.stroke();});
    g.beginPath();g.moveTo(0,cy);g.lineTo(W,cy);g.moveTo(cx,0);g.lineTo(cx,H);g.stroke();

    const rad=deg*Math.PI/180;
    const tx=cx+Math.sin(rad)*sc*0.72, ty=cy-Math.cos(rad)*sc*0.72;
    const bx=cx-Math.sin(rad)*sc*0.28, by=cy+Math.cos(rad)*sc*0.28;

    // ghost trail
    if (trail.length>1) {
      g.beginPath();
      trail.forEach((a,i)=>{
        const r2=a*Math.PI/180;
        i===0?g.moveTo(cx+Math.sin(r2)*sc*1.18, cy-Math.cos(r2)*sc*1.18)
             :g.lineTo(cx+Math.sin(r2)*sc*1.18, cy-Math.cos(r2)*sc*1.18);
      });
      g.strokeStyle="rgba(252,92,124,0.13)"; g.lineWidth=5*dpr; g.stroke();
    }

    // arm
    g.beginPath();g.moveTo(bx,by);g.lineTo(tx,ty);
    g.strokeStyle="#1d0d35";g.lineWidth=12*dpr;g.lineCap="round";g.stroke();
    g.beginPath();g.moveTo(bx,by);g.lineTo(tx,ty);
    g.strokeStyle="#7c5cfc";g.lineWidth=4*dpr;g.stroke();

    // counterweight
    g.beginPath();g.arc(bx,by,8*dpr,0,Math.PI*2);
    g.fillStyle="#2a1845";g.fill();g.strokeStyle="#7c5cfc";g.lineWidth=1.5*dpr;g.stroke();

    // chain with lag
    const lagDeg=trail.length>10?deg+(trail[trail.length-10]-deg)*0.6:deg;
    const lr=lagDeg*Math.PI/180;
    const cex=cx+Math.sin(lr)*sc*1.18, cey=cy-Math.cos(lr)*sc*1.18;
    g.beginPath();g.moveTo(tx,ty);
    g.quadraticCurveTo(tx+(cex-tx)*0.18, ty+(cey-ty)*0.72, cex,cey);
    g.strokeStyle="#fc5c7c";g.lineWidth=2.5*dpr;g.setLineDash([5*dpr,3*dpr]);g.stroke();g.setLineDash([]);
    g.beginPath();g.arc(cex,cey,5.5*dpr,0,Math.PI*2);g.fillStyle="#fc5c7c";g.fill();

    // hub
    g.beginPath();g.arc(cx,cy,10*dpr,0,Math.PI*2);
    g.fillStyle="#0e0e1a";g.fill();g.strokeStyle="#4a4a7a";g.lineWidth=2*dpr;g.stroke();

    // labels
    const disp=((deg%360)+360)%360;
    g.fillStyle="#7c5cfc";g.font=`${Math.round(13*dpr)}px monospace`;
    g.fillText(`${disp.toFixed(1)}°`,10*dpr,20*dpr);
    if (Math.abs(deg)>360) {
      g.fillStyle="#fc9c4c";g.font=`${Math.round(11*dpr)}px monospace`;
      g.fillText(`${(deg/360).toFixed(1)} rev`,10*dpr,35*dpr);
    }
  }, [deg, trail]);
  return <canvas ref={ref} style={{width:"100%",height:"100%",display:"block",
    borderRadius:8,background:"#07070e",border:"1px solid #181828"}}/>;
}

// ─── Timeline canvas ──────────────────────────────────────────────────────────
const BLK_COLS = ["#7c5cfc","#5c9cfc","#fc7c5c","#5cfca0","#fc9c5c","#c05cfc"];

function TLCanvas({ sim, marker }) {
  const ref = useRef();
  useEffect(()=>{
    const cv=ref.current; if(!cv||!sim) return;
    const dpr=window.devicePixelRatio||1;
    const {width:rw,height:rh}=cv.getBoundingClientRect();
    cv.width=rw*dpr; cv.height=rh*dpr;
    const g=cv.getContext("2d");
    g.clearRect(0,0,cv.width,cv.height);
    const PAD=40*dpr,PW=cv.width-PAD*2,PH=cv.height-20*dpr,dur=sim.dur||1;

    let lo=-20,hi=20;
    sim.ev.forEach(e=>{if(e.type==="move"||e.type==="spin"){lo=Math.min(lo,e.p0,e.p1);hi=Math.max(hi,e.p0,e.p1);}});
    const pad=(hi-lo)*0.1; lo-=pad; hi+=pad; const aR=hi-lo;
    const tx=t=>PAD+(t/dur)*PW, ay=a=>PH*0.05+PH*0.9*(1-(a-lo)/aR);

    // zero line
    g.strokeStyle="#1e1e32";g.lineWidth=1;
    g.beginPath();g.moveTo(PAD,ay(0));g.lineTo(PAD+PW,ay(0));g.stroke();

    sim.ev.forEach(e=>{
      const col=BLK_COLS[(e.id??0)%BLK_COLS.length];
      if(e.type==="move"){
        if(e.risk){g.fillStyle="rgba(252,92,124,0.09)";g.fillRect(tx(e.t0),0,tx(e.t1)-tx(e.t0),cv.height);}
        g.strokeStyle=e.risk?"#fc5c7c":col; g.lineWidth=e.risk?3:2;
        g.beginPath();g.moveTo(tx(e.t0),ay(e.p0));g.lineTo(tx(e.t1),ay(e.p1));g.stroke();
      } else if(e.type==="spin"){
        g.strokeStyle="#fc9c4c";g.lineWidth=2;g.setLineDash([5*dpr,3*dpr]);
        g.beginPath();g.moveTo(tx(e.t0),ay(e.p0));g.lineTo(tx(e.t1),ay(e.p1));g.stroke();
        g.setLineDash([]);
        const mx=tx((e.t0+e.t1)/2),my=ay((e.p0+e.p1)/2);
        g.fillStyle="#fc9c4c";g.font=`${11*dpr}px monospace`;
        g.fillText(e.dir>0?"↻":"↺",mx-6*dpr,my);
      } else if(e.type==="wait"){
        g.strokeStyle="#22223a";g.lineWidth=1.5;g.setLineDash([3*dpr,3*dpr]);
        g.beginPath();g.moveTo(tx(e.t0),ay(e.pos));g.lineTo(tx(e.t1),ay(e.pos));g.stroke();
        g.setLineDash([]);
      }
    });

    // time axis
    const step=dur<=12?1:dur<=60?5:15;
    g.fillStyle="#333355";g.font=`${8*dpr}px monospace`;
    for(let tt=0;tt<=dur;tt+=step){
      const x=tx(tt);
      g.strokeStyle="#141422";g.lineWidth=1;g.beginPath();g.moveTo(x,0);g.lineTo(x,PH);g.stroke();
      g.fillText(`${tt}s`,x-4*dpr,cv.height-4*dpr);
    }

    // playhead
    if(marker>0){
      g.strokeStyle="rgba(200,200,255,0.5)";g.lineWidth=1.5;
      const mx=tx(Math.min(marker,dur));
      g.beginPath();g.moveTo(mx,0);g.lineTo(mx,cv.height);g.stroke();
    }
  },[sim,marker]);
  return <canvas ref={ref} style={{width:"100%",height:"100%",display:"block",
    borderRadius:6,background:"#07070e",border:"1px solid #181828"}}/>;
}

// ─── Tiny components ──────────────────────────────────────────────────────────
function Slider({label,value,min,max,step=1,fmt=v=>v,onChange}) {
  return (
    <div style={{display:"flex",flexDirection:"column",gap:3}}>
      <div style={{display:"flex",justifyContent:"space-between"}}>
        <span style={{fontFamily:"monospace",fontSize:9,letterSpacing:"0.12em",textTransform:"uppercase",color:"#4a4a6a"}}>{label}</span>
        <span style={{fontFamily:"monospace",fontSize:11,color:"#d0d0f0"}}>{fmt(value)}</span>
      </div>
      <input type="range" min={min} max={max} step={step} value={value}
        onChange={e=>onChange(Number(e.target.value))}
        style={{width:"100%",accentColor:"#7c5cfc",cursor:"pointer"}}/>
    </div>
  );
}

function Tog({label,on,set}) {
  return (
    <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",fontSize:12,color:"#b0b0d0"}}>
      <span>{label}</span>
      <div onClick={()=>set(!on)} style={{position:"relative",width:32,height:18,borderRadius:9,
        background:on?"#7c5cfc":"#22223a",cursor:"pointer",transition:"background .15s",flexShrink:0}}>
        <div style={{position:"absolute",width:12,height:12,borderRadius:"50%",background:"#07070e",
          top:3,left:on?17:3,transition:"left .15s"}}/>
      </div>
    </div>
  );
}

const VCOLS = {
  d:["#0e0e1c","#22223a","#9090b8"], a:["#180e38","#7c5cfc","#9c7cff"],
  g:["#081a10","#2c9c5c","#3ccc7c"], r:["#1e0808","#9c3c4c","#fc5c7c"],
  o:["#1a1008","#9c6c2c","#fc9c4c"],
};
function Btn({children,onClick,v="d",sm=false,style:sx={}}) {
  const [bg,br,fg]=VCOLS[v]||VCOLS.d;
  return <div onClick={onClick} style={{flex:1,padding:sm?"3px 9px":"7px 6px",border:`1px solid ${br}`,
    borderRadius:6,background:bg,color:fg,fontFamily:"monospace",fontSize:sm?10:11,
    cursor:"pointer",textAlign:"center",userSelect:"none",...sx}}>{children}</div>;
}

// ─── Block card ───────────────────────────────────────────────────────────────
const ICONS = {pendulum:"⇌",wave:"〜",spin:"↻",buildup:"↑",pause:"⏸",swing:"→"};

function BlockCard({blk,colIdx,selected,hasRisk,G,onSelect,onRemove,onChange}) {
  const col = BLK_COLS[colIdx % BLK_COLS.length];
  const ang  = blk.angle  ?? G.angle;
  const rest = blk.restMs ?? G.restMs;
  const spd  = blk.speed  ?? G.speed;

  const spdLabel = blk.speedEnd != null
    ? `${spd}→${blk.speedEnd}sps`
    : `${spd}sps`;
  const summary = {
    pendulum: `${blk.reps}× · ${ang}° · ${spdLabel}`,
    wave:     `${blk.reps}× · ${ang}° · ${rest}ms`,
    spin:     `${blk.continuous?"∞":(blk.revs??2)} rev · ${blk.dir==="both"?"CW+CCW":blk.dir>0?"CW":"CCW"} · ${spd}sps`,
    buildup:  `${blk.reps}× to ${ang}°`,
    pause:    `${((blk.ms??1000)/1000).toFixed(1)}s`,
    swing:    `${(blk.dir??1)>0?"→ Right":"← Left"} · ${ang}°`,
  }[blk.type] || "";

  const upd = patch => onChange({...blk,...patch});

  return (
    <div onClick={onSelect} style={{borderRadius:8,border:`1px solid ${selected?col:"#1c1c2c"}`,
      background:selected?"#0e0c1a":"#0a0a14",borderLeft:`3px solid ${hasRisk?"#fc5c7c":col}`,
      padding:"8px 10px",cursor:"pointer",userSelect:"none",
      boxShadow:selected?`0 0 14px ${col}28`:"none",transition:"border-color .15s"}}>

      <div style={{display:"flex",alignItems:"center",gap:7}}>
        <span style={{color:col,fontFamily:"monospace",fontSize:13}}>{ICONS[blk.type]}</span>
        <span style={{fontFamily:"monospace",fontSize:11,color:col,flex:1,textTransform:"capitalize"}}>{blk.type}</span>
        <span style={{fontFamily:"monospace",fontSize:9,color:"#333355"}}>{summary}</span>
        {hasRisk&&<span style={{fontSize:10,color:"#fc5c7c"}}>⚠</span>}
        <span onClick={e=>{e.stopPropagation();onRemove();}}
          style={{color:"#2a2a42",fontSize:14,padding:"0 3px",cursor:"pointer",lineHeight:1}}>✕</span>
      </div>

      {selected && (
        <div onClick={e=>e.stopPropagation()}
          style={{marginTop:10,paddingTop:9,borderTop:"1px solid #1c1c2c",
            display:"flex",flexDirection:"column",gap:9}}>

          {/* Per-type controls */}
          {(blk.type==="pendulum"||blk.type==="wave"||blk.type==="buildup")&&
            <Slider label="Repetitions" value={blk.reps} min={1} max={16} fmt={v=>`${v}×`}
              onChange={v=>upd({reps:v})}/>}

          {blk.type==="wave"&&
            <Tog label="Constant speed (no buildup)" on={!!blk.constant} set={v=>upd({constant:v})}/>}

          {blk.type==="pendulum"&&<>
            <Tog label="Offset drift (one-sided)" on={!!blk.oneSided} set={v=>upd({oneSided:v})}/>
            {blk.oneSided&&<Slider label="Drift ratio (0=max drift, 1=full return)" value={blk.driftRatio??0.5}
              min={0} max={1} step={0.05} fmt={v=>`${Math.round((1-v)*100)}% drift`}
              onChange={v=>upd({driftRatio:v})}/>}
          </>}

          {blk.type==="spin"&&<>
            <Tog label="Continuous (∞)" on={!!blk.continuous} set={v=>upd({continuous:v})}/>
            {!blk.continuous&&<Slider label="Revolutions" value={blk.revs??2} min={1} max={12} fmt={v=>`${v} rev`}
              onChange={v=>upd({revs:v})}/>}
            <div>
              <div style={{fontFamily:"monospace",fontSize:9,letterSpacing:"0.12em",
                textTransform:"uppercase",color:"#4a4a6a",marginBottom:5}}>Direction</div>
              <div style={{display:"flex",gap:5}}>
                <Btn sm v={blk.dir===1?"a":"d"} onClick={()=>upd({dir:1})}>↻ CW</Btn>
                <Btn sm v={blk.dir===-1?"a":"d"} onClick={()=>upd({dir:-1})}>↺ CCW</Btn>
                <Btn sm v={blk.dir==="both"?"a":"d"} onClick={()=>upd({dir:"both"})}>↻↺ Both</Btn>
              </div>
            </div>
          </>}

          {blk.type==="swing"&&(
            <div>
              <div style={{fontFamily:"monospace",fontSize:9,letterSpacing:"0.12em",
                textTransform:"uppercase",color:"#4a4a6a",marginBottom:5}}>Direction</div>
              <div style={{display:"flex",gap:5}}>
                <Btn sm v={(blk.dir??1)===1?"a":"d"} onClick={()=>upd({dir:1})}>→ Right</Btn>
                <Btn sm v={(blk.dir??1)===-1?"a":"d"} onClick={()=>upd({dir:-1})}>← Left</Btn>
              </div>
            </div>
          )}

          {blk.type==="pause"&&
            <Slider label="Duration" value={blk.ms??1000} min={100} max={10000} step={100}
              fmt={v=>`${(v/1000).toFixed(1)}s`} onChange={v=>upd({ms:v})}/>}

          {/* Block overrides */}
          <div style={{paddingTop:7,borderTop:"1px solid #161626",display:"flex",flexDirection:"column",gap:7}}>
            <div style={{fontFamily:"monospace",fontSize:9,color:"#2a2a42",letterSpacing:"0.1em",textTransform:"uppercase"}}>
              Override globals for this block
            </div>
            <div style={{display:"flex",gap:5}}>
              {[["angle","ang",5,360,5,v=>`${v}°`],
                ["speed","spd",100,8000,100,v=>`${v}sps`],
                ["restMs","rest",0,3000,50,v=>`${v}ms`]
              ].map(([key,lbl,_min,_max,_step,fmt])=>(
                <Btn key={key} sm v={blk[key]!=null?"a":"d"}
                  onClick={()=>upd({[key]:blk[key]!=null?null:G[key]})}>
                  {blk[key]!=null?`${lbl}:${fmt(blk[key])}`:`+ ${lbl}`}
                </Btn>
              ))}
            </div>
            {blk.angle!=null&&<Slider label="Angle" value={blk.angle} min={5} max={360} step={5}
              fmt={v=>`${v}°`} onChange={v=>upd({angle:v})}/>}
            {blk.speed!=null&&<Slider label="Speed start" value={blk.speed} min={100} max={3000} step={100}
              fmt={v=>`${v} sps`} onChange={v=>upd({speed:v})}/>}
            {blk.speed!=null&&blk.type==="pendulum"&&(
              <div style={{display:"flex",flexDirection:"column",gap:5}}>
                <Tog label="Speed ramp (end speed)"
                  on={blk.speedEnd!=null}
                  set={v=>upd({speedEnd: v ? (blk.speed ?? G.speed) : null})}/>
                {blk.speedEnd!=null&&
                  <Slider label="Speed end" value={blk.speedEnd} min={100} max={3000} step={100}
                    fmt={v=>`${v} sps`} onChange={v=>upd({speedEnd:v})}/>}
              </div>
            )}
            {blk.restMs!=null&&<Slider label="Rest" value={blk.restMs} min={0} max={3000} step={50}
              fmt={v=>v===0?"none":`${v}ms`} onChange={v=>upd({restMs:v})}/>}
            <div style={{display:"flex",gap:5}}>
              <Btn sm v={blk.brk!=null?"a":"d"}
                onClick={()=>upd({brk: blk.brk!=null ? null : G.brk})}>
                {blk.brk!=null ? `brk:${blk.brk?"on":"off"}` : "+ brk"}
              </Btn>
              <Btn sm v={blk.soft!=null?"a":"d"}
                onClick={()=>upd({soft: blk.soft!=null ? null : G.soft})}>
                {blk.soft!=null ? `soft:${blk.soft?"on":"off"}` : "+ soft"}
              </Btn>
            </div>
            {blk.brk!=null&&<Tog label="Braking" on={!!blk.brk} set={v=>upd({brk:v})}/>}
            {blk.soft!=null&&<Tog label="Soft-start" on={!!blk.soft} set={v=>upd({soft:v})}/>}
            <Tog label="Invert this block" on={!!blk.invert} set={v=>upd({invert:v})}/>
          </div>
        </div>
      )}
    </div>
  );
}

// ─── Main ─────────────────────────────────────────────────────────────────────
let _id = 1;
const mkBlk = (type,extra={}) => ({
  id:_id++, type, reps:4, revs:2, dir:1, ms:1000,
  angle:null, speed:null, restMs:null, ...extra
});

export default function App() {
  // globals
  const [speed,     setSpeed]     = useState(2000);
  const [angle,     setAngle]     = useState(90);
  const [restMs,    setRestMs]    = useState(500);
  const [brk,       setBrk]       = useState(true);
  const [soft,      setSoft]      = useState(true);
  const [angleMult,       setAngleMult]       = useState(1.0);
  const [restMult,        setRestMult]        = useState(1.0);
  const [invert, setInvert] = useState(false);
  const G = { speed, angle, restMs, brk, soft, angleMult, restMult, invert };

  // sequence
  const [blocks,  setBlocks]  = useState([]);
  const [selIdx,  setSelIdx]  = useState(-1);
  const [sim,     setSim]     = useState(null);
  const [seqName, setSeqName] = useState("");
  const [saved,   setSaved]   = useState(() => {
    try { return JSON.parse(localStorage.getItem("whip_saved") || "{}"); }
    catch { return {}; }
  });
  const [piSyncing, setPiSyncing] = useState(false);
  const [tab,     setTab]     = useState("compose");

  // Pi4 connection
  const [piUrl,    setPiUrl]    = useState("http://10.42.0.1:8080");
  const [piStatus, setPiStatus] = useState(null); // null = unreachable
  const [loopMode, setLoopMode] = useState(true);
  const [piErr,    setPiErr]    = useState("");
  const [piRunning,setPiRunning]= useState(false);

  // clear confirm
  const [clearing, setClearing] = useState(false);
  const clearT = useRef(null);

  // live sync
  const [liveSync,   setLiveSync]   = useState(true);   // toggle
  const [syncPending,setSyncPending] = useState(false);  // visual indicator
  const syncTimer = useRef(null);
  const piStatusRef = useRef(null);
  useEffect(() => { piStatusRef.current = piStatus; }, [piStatus]);
  useEffect(() => { localStorage.setItem("whip_saved", JSON.stringify(saved)); }, [saved]);

  // animation
  const [animDeg, setAnimDeg] = useState(0);
  const [trail,   setTrail]   = useState([]);
  const [marker,  setMarker]  = useState(0);
  const rafRef = useRef(null), t0Ref = useRef(null), simRef = useRef(null);

  const stopAnim = useCallback(() => {
    if (rafRef.current) { cancelAnimationFrame(rafRef.current); rafRef.current=null; }
  }, []);

  const startAnim = useCallback((s) => {
    stopAnim(); simRef.current=s; t0Ref.current=null; setTrail([]);
    const tick = ts => {
      if (!t0Ref.current) t0Ref.current=ts;
      const s2=simRef.current; if(!s2?.dur) return;
      const lt=((ts-t0Ref.current)/1000)%(s2.dur+1.5);
      const p=posAt(s2.ev,lt);
      setAnimDeg(p);
      setMarker(Math.min(lt,s2.dur));
      setTrail(h=>{const n=[...h,p];return n.length>60?n.slice(-60):n;});
      rafRef.current=requestAnimationFrame(tick);
    };
    rafRef.current=requestAnimationFrame(tick);
  },[stopAnim]);

  useEffect(()=>()=>stopAnim(),[stopAnim]);

  // Pi4 status polling
  useEffect(()=>{
    if (!piUrl) return;
    let cancelled = false;
    const poll = async () => {
      try {
        const r = await fetch(`${piUrl}/api/status`,
          { signal: AbortSignal.timeout(2000) });
        const d = await r.json();
        if (!cancelled) { setPiStatus(d); setPiErr(""); }
      } catch {
        if (!cancelled) setPiStatus(null);
      }
    };
    poll();
    const id = setInterval(poll, 1500);
    return () => { cancelled = true; clearInterval(id); };
  }, [piUrl]);

  const runOnPi = async () => {
    if (!blocks.length) return;
    const n = (seqName.trim() || "sequence").replace(/\//g,"-");
    const steps = compileToSteps(blocks, G);
    setPiRunning(true); setPiErr("");
    try {
      await fetch(`${piUrl}/api/sequences`, {
        method:"POST", headers:{"Content-Type":"application/json"},
        // Include blocks+globals so the Pi4 copy is fully round-trippable
        body: JSON.stringify({name:n, steps, blocks, globals:G}),
        signal: AbortSignal.timeout(8000),
      });
      await fetch(`${piUrl}/api/sequences/${encodeURIComponent(n)}/run`, {
        method:"POST", headers:{"Content-Type":"application/json"},
        body: JSON.stringify({loop: loopMode}),
        signal: AbortSignal.timeout(8000),
      });
      // Auto-save to local library whenever we push to Pi4
      setSaved(s => ({...s, [n]: {name:n, blocks:[...blocks], G}}));
    } catch(e) { setPiErr(e.message ?? "timeout — is the Pi4 server running?"); }
    finally { setPiRunning(false); }
  };

  const stopMotor = async () => {
    setPiErr("");
    try { await fetch(`${piUrl}/api/stop`, {
      method:"POST", signal: AbortSignal.timeout(3000),
    }); }
    catch(e) { setPiErr(e.message ?? "timeout"); }
  };

  // re-simulate whenever blocks or globals change
  useEffect(()=>{
    if (!blocks.length) { setSim(null); stopAnim(); setAnimDeg(0); setTrail([]); return; }
    const s = simulate(blocks, G);
    setSim(s); simRef.current=s;
    startAnim(s);
  }, [blocks, speed, angle, restMs, brk, soft, angleMult, restMult, invert]);

  // Auto-sync: when blocks/globals change and engine is already running, re-upload after debounce
  useEffect(() => {
    if (!liveSync || !blocks.length) return;
    clearTimeout(syncTimer.current);
    syncTimer.current = setTimeout(async () => {
      const st = piStatusRef.current;
      if (!st?.engine?.running) return;
      const n = st.engine.name;
      // Only sync if we're editing the sequence that's actually running —
      // otherwise we'd silently overwrite a different sequence (e.g. autostart)
      // with whatever happens to be in the compose view.
      if ((seqName.trim() || "sequence").replace(/\//g,"-") !== n) return;
      const steps = compileToSteps(blocks, G);
      setSyncPending(true);
      try {
        await fetch(`${piUrl}/api/sequences`, {
          method:"POST", headers:{"Content-Type":"application/json"},
          body: JSON.stringify({name:n, steps}),
          signal: AbortSignal.timeout(5000),
        });
        await fetch(`${piUrl}/api/sequences/${encodeURIComponent(n)}/run`, {
          method:"POST", headers:{"Content-Type":"application/json"},
          body: JSON.stringify({loop: st.engine.loop ?? loopMode}),
          signal: AbortSignal.timeout(5000),
        });
      } catch {} finally { setSyncPending(false); }
    }, 800);
    return () => clearTimeout(syncTimer.current);
  }, [blocks, speed, angle, restMs, brk, soft, angleMult, restMult, invert, liveSync, piUrl, loopMode, seqName]);

  const addBlock = type => {
    const nb = mkBlk(type);
    setBlocks(b=>{setSelIdx(b.length); return [...b,nb];});
  };

  const updBlk = (i,v)=>setBlocks(b=>b.map((x,j)=>j===i?v:x));
  const remBlk = i=>{setBlocks(b=>b.filter((_,j)=>j!==i));setSelIdx(s=>s>=i?Math.max(-1,s-1):s);};
  const moveUp = i=>{if(i<1)return;setBlocks(b=>{const n=[...b];[n[i-1],n[i]]=[n[i],n[i-1]];return n;});setSelIdx(i-1);};
  const moveDn = i=>{if(i>=blocks.length-1)return;setBlocks(b=>{const n=[...b];[n[i],n[i+1]]=[n[i+1],n[i]];return n;});setSelIdx(i+1);};

  const clearAll = () => {
    if (!blocks.length) return;
    if (!clearing) { setClearing(true); clearT.current=setTimeout(()=>setClearing(false),2500); return; }
    clearTimeout(clearT.current); setClearing(false);
    setBlocks([]); setSelIdx(-1); setSim(null); setAnimDeg(0); setTrail([]);
  };

  const saveSeq = ()=>{const n=seqName.trim()||"untitled";setSaved(s=>({...s,[n]:{name:n,blocks:[...blocks],G}}));};
  const backupSeq = ()=>{
    if(!blocks.length)return;
    const base=seqName.trim()||"untitled";
    const ts=new Date().toISOString().slice(11,19).replace(/:/g,"");
    const n=`${base}_${ts}`;
    setSaved(s=>({...s,[n]:{name:n,blocks:[...blocks],G}}));
  };
  const loadSeq = n=>{const d=saved[n];if(!d)return;setBlocks(d.blocks||[]);setSeqName(d.name);
    const g = d.G ?? d.globals;
    if(g){setSpeed(g.speed);setAngle(g.angle);setRestMs(g.restMs);setBrk(g.brk);setSoft(g.soft);
      if(g.angleMult!=null)setAngleMult(g.angleMult);
      if(g.restMult!=null)setRestMult(g.restMult);
      if(g.invert!=null)setInvert(g.invert);}
    setSelIdx(-1);setTab("compose");};

  const syncFromPi = async () => {
    setPiSyncing(true);
    try {
      const listRes = await fetch(`${piUrl}/api/sequences`, {signal:AbortSignal.timeout(5000)});
      const names = await listRes.json();
      const entries = await Promise.all(names.map(async nm => {
        try {
          const r = await fetch(`${piUrl}/api/sequences/${encodeURIComponent(nm)}`,
            {signal:AbortSignal.timeout(5000)});
          return await r.json();
        } catch { return null; }
      }));
      setSaved(s => {
        const next = {...s};
        for (const d of entries) {
          if (!d) continue;
          const g = d.globals ?? d.G;
          next[d.name] = {name:d.name, blocks:d.blocks||[], G:g||null};
        }
        return next;
      });
    } catch(e) { setPiErr(e.message ?? "sync failed"); }
    finally { setPiSyncing(false); }
  };

  const exportJSON = ()=>{
    const n=seqName.trim()||"sequence";
    const b=new Blob([JSON.stringify({name:n,blocks,globals:G},null,2)],{type:"application/json"});
    const a=document.createElement("a");a.href=URL.createObjectURL(b);a.download=`${n}.json`;a.click();
  };
  const importJSON = e=>{
    const f=e.target.files[0];if(!f)return;
    const r=new FileReader();
    r.onload=ev=>{try{const d=JSON.parse(ev.target.result);
      if(d.blocks)setBlocks(d.blocks);if(d.name)setSeqName(d.name);
      if(d.globals){setSpeed(d.globals.speed);setAngle(d.globals.angle);
        setRestMs(d.globals.restMs);setBrk(d.globals.brk);setSoft(d.globals.soft);
        if(d.globals.angleMult!=null)setAngleMult(d.globals.angleMult);
        if(d.globals.restMult!=null)setRestMult(d.globals.restMult);
        if(d.globals.invert!=null)setInvert(d.globals.invert);}
    }catch{alert("Invalid JSON");}};r.readAsText(f);
  };

  const risks = sim?.ev.filter(e=>e.risk).length ?? 0;
  const dur   = sim?.dur ?? 0;

  // ─── Layout ───────────────────────────────────────────────────────────────
  // Fill the iframe completely
  useEffect(()=>{
    document.body.style.cssText="margin:0;padding:0;overflow:hidden;height:100%;";
    document.documentElement.style.cssText="height:100%;";
  },[]);
  const lbl={fontFamily:"monospace",fontSize:9,letterSpacing:"0.13em",textTransform:"uppercase",
    color:MUT,marginBottom:4,display:"block"};
  const card={background:SURF,border:`1px solid ${BDR}`,borderRadius:8,padding:10,
    display:"flex",flexDirection:"column",gap:8};

  return (
    <div style={{position:"absolute",inset:0,display:"flex",overflow:"hidden",
      background:BG,color:"#b8b8d8",fontFamily:"system-ui,sans-serif",fontSize:13}}>

      {/* ── Left panel ─────────────────────────────────── */}
      <div style={{width:300,flexShrink:0,display:"flex",flexDirection:"column",
        borderRight:`1px solid ${BDR}`,overflow:"hidden"}}>

        {/* header */}
        <div style={{padding:"10px 12px",borderBottom:`1px solid ${BDR}`,background:SURF,flexShrink:0,
          display:"flex",alignItems:"center",gap:8}}>
          <span style={{fontFamily:"monospace",fontSize:10,letterSpacing:"0.15em",
            textTransform:"uppercase",color:MUT,flex:1}}>Choreographer</span>
          {sim&&<span style={{fontFamily:"monospace",fontSize:10,color:risks?"#fc5c7c":"#3ccc7c"}}>
            {dur.toFixed(1)}s{risks?` ⚠${risks}`:""}</span>}
        </div>

        {/* tabs */}
        <div style={{display:"flex",borderBottom:`1px solid ${BDR}`,flexShrink:0}}>
          {["compose","library"].map(t=>(
            <div key={t} onClick={()=>setTab(t)} style={{flex:1,padding:"8px 4px",textAlign:"center",
              fontFamily:"monospace",fontSize:10,letterSpacing:"0.08em",textTransform:"uppercase",
              cursor:"pointer",color:tab===t?"#d8d8f8":MUT,
              borderBottom:tab===t?"2px solid #7c5cfc":"2px solid transparent",marginBottom:-1}}>
              {t}
            </div>
          ))}
        </div>

        {/* scrollable content */}
        <div style={{flex:1,overflowY:"auto",padding:11,display:"flex",flexDirection:"column",gap:10}}>

          {tab==="compose"&&<>
            {/* globals */}
            <div style={card}>
              <span style={lbl}>Global parameters</span>
              <Slider label="Speed" value={speed} min={100} max={3000} step={100}
                fmt={v=>`${v} sps · ${Math.round(v/400*60/5)} rpm`} onChange={setSpeed}/>
              <Slider label="Swing angle" value={angle} min={5} max={360} step={5}
                fmt={v=>`${v}°`} onChange={setAngle}/>
              <Slider label="Rest between swings" value={restMs} min={0} max={3000} step={50}
                fmt={v=>v===0?"none":`${v} ms`} onChange={setRestMs}/>
              <Slider label="Angle multiplier" value={angleMult} min={0.1} max={3} step={0.05}
                fmt={v=>`×${v.toFixed(2)}`} onChange={setAngleMult}/>
              <Slider label="Rest multiplier" value={restMult} min={0} max={2} step={0.05}
                fmt={v=>`×${v.toFixed(2)}`} onChange={setRestMult}/>
              <div style={{borderTop:`1px solid ${BDR}`,paddingTop:8,display:"flex",flexDirection:"column",gap:6}}>
                <Tog label="Braking" on={brk} set={setBrk}/>
                <Tog label="Soft-start" on={soft} set={setSoft}/>
                <Tog label="Invert all directions" on={invert} set={setInvert}/>
              </div>
            </div>

            {/* add blocks */}
            <div>
              <span style={lbl}>Add block</span>
              <div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:5}}>
                {[["pendulum","⇌ Pendulum"],["wave","〜 Wave"],
                  ["spin","↻ Spin"],["buildup","↑ Build-up"],
                  ["swing","→ Swing"],["pause","⏸ Pause"]
                ].map(([type,label])=>(
                  <Btn key={type} onClick={()=>addBlock(type)}>{label}</Btn>
                ))}
              </div>
            </div>

            {/* block list */}
            <div style={{display:"flex",flexDirection:"column",gap:5}}>
              <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",marginBottom:2}}>
                <span style={lbl}>Sequence ({blocks.length} blocks)</span>
                <div style={{display:"flex",gap:4}}>
                  {selIdx>=0&&<><Btn sm onClick={()=>moveUp(selIdx)}>↑</Btn>
                    <Btn sm onClick={()=>moveDn(selIdx)}>↓</Btn></>}
                  <Btn sm v={clearing?"r":"d"} onClick={clearAll}>{clearing?"CONFIRM?":"CLEAR"}</Btn>
                </div>
              </div>

              {blocks.length===0&&(
                <div style={{fontFamily:"monospace",fontSize:10,color:"#1e1e30",
                  padding:14,textAlign:"center",border:`1px dashed #1c1c2c`,borderRadius:8}}>
                  Click a block above to start.<br/>Globals apply to everything.
                </div>
              )}

              {blocks.map((blk,i)=>(
                <BlockCard key={blk.id} blk={blk} colIdx={i}
                  selected={selIdx===i}
                  hasRisk={sim?.ev.some(e=>e.id===blk.id&&e.risk)}
                  G={G}
                  onSelect={()=>setSelIdx(selIdx===i?-1:i)}
                  onRemove={()=>remBlk(i)}
                  onChange={v=>updBlk(i,v)}/>
              ))}
            </div>

            {/* save / export */}
            <div style={{display:"flex",flexDirection:"column",gap:5,marginTop:"auto",paddingTop:6}}>
              <div style={{display:"flex",gap:5}}>
                <input value={seqName} onChange={e=>setSeqName(e.target.value)}
                  placeholder="sequence name"
                  style={{flex:1,background:SURF,border:`1px solid ${BDR}`,color:"#c0c0e0",
                    borderRadius:5,padding:"5px 7px",fontFamily:"monospace",fontSize:11}}/>
                <Btn sm v="g" onClick={saveSeq}>SAVE</Btn>
                <Btn sm v="o" onClick={backupSeq} style={{opacity:blocks.length?1:0.35}}>⎘ BKP</Btn>
              </div>
              <div style={{display:"flex",gap:5}}>
                <Btn sm onClick={exportJSON}>↓ JSON</Btn>
                <Btn sm v="o" onClick={()=>{
                  const n=seqName.trim()||"sequence";
                  const steps=compileToSteps(blocks,G);
                  const b=new Blob([JSON.stringify({name:n,steps},null,2)],{type:"application/json"});
                  const a=document.createElement("a");a.href=URL.createObjectURL(b);
                  a.download=`${n}_pi4.json`;a.click();
                }}>↓ Pi4</Btn>
                <label style={{flex:1,padding:"3px 8px",border:`1px solid ${BDR}`,borderRadius:6,
                  background:SURF,color:MUT,fontFamily:"monospace",fontSize:10,
                  cursor:"pointer",textAlign:"center",userSelect:"none"}}>
                  ↑ IMPORT<input type="file" accept=".json" style={{display:"none"}} onChange={importJSON}/>
                </label>
              </div>
            </div>
          </>}

          {tab==="library"&&<>
            <div style={{display:"flex",alignItems:"center",justifyContent:"space-between"}}>
              <span style={lbl}>Saved sequences</span>
              <Btn sm v="a" onClick={syncFromPi} style={{opacity:piSyncing?0.5:1}}>
                {piSyncing?"syncing…":"↓ Sync Pi4"}
              </Btn>
            </div>
            {Object.keys(saved).length===0
              ? <div style={{fontFamily:"monospace",fontSize:10,color:"#1e1e30"}}>Nothing saved yet. Hit "↓ Sync Pi4" to pull from the Pi.</div>
              : Object.keys(saved).sort().map(n=>(
                <div key={n} style={{display:"flex",alignItems:"center",gap:5,padding:"7px 9px",
                  border:`1px solid ${BDR}`,borderRadius:6,background:SURF}}>
                  <div style={{flex:1,minWidth:0}}>
                    <div style={{fontFamily:"monospace",fontSize:11,whiteSpace:"nowrap",overflow:"hidden",textOverflow:"ellipsis"}}>{n}</div>
                    {saved[n]?.blocks?.length
                      ? <div style={{fontFamily:"monospace",fontSize:9,color:"#3ccc7c"}}>{saved[n].blocks.length} blocks</div>
                      : <div style={{fontFamily:"monospace",fontSize:9,color:MUT}}>steps only</div>}
                  </div>
                  <Btn sm onClick={()=>loadSeq(n)} style={{opacity:saved[n]?.blocks?.length?1:0.4,
                    cursor:saved[n]?.blocks?.length?"pointer":"not-allowed"}}>LOAD</Btn>
                  <Btn sm v="r" onClick={()=>setSaved(s=>{const x={...s};delete x[n];return x;})}>✕</Btn>
                </div>
              ))
            }
          </>}
        </div>
      </div>

      {/* ── Pi4 control footer ──────────────────────────── */}
      <div style={{borderTop:`1px solid ${BDR}`,padding:"8px 10px",flexShrink:0,
        background:SURF,display:"flex",flexDirection:"column",gap:6}}>

        {/* URL + online dot */}
        <div style={{display:"flex",alignItems:"center",gap:6}}>
          <input value={piUrl} onChange={e=>setPiUrl(e.target.value)}
            style={{flex:1,background:BG,border:`1px solid ${BDR}`,color:"#c0c0e0",
              borderRadius:5,padding:"4px 7px",fontFamily:"monospace",fontSize:10}}/>
          <div title={piStatus?"online":"offline"} style={{width:8,height:8,borderRadius:"50%",flexShrink:0,
            background:piStatus?"#3ccc7c":"#2a2a42",
            boxShadow:piStatus?"0 0 5px #3ccc7c":""}}/>
        </div>

        {/* Engine status */}
        {piStatus?.engine?.running&&(
          <div style={{fontFamily:"monospace",fontSize:9,color:"#fc9c4c",letterSpacing:"0.05em"}}>
            ▶ {piStatus.engine.name}
            {" · "}{piStatus.engine.step}/{piStatus.engine.total}
            {piStatus.engine.loop?" · ∞ loop":""}
          </div>
        )}

        {/* Loop + live-sync toggles */}
        <div style={{display:"flex",flexDirection:"column",gap:5}}>
          <Tog label="Loop" on={loopMode} set={setLoopMode}/>
          <div style={{display:"flex",alignItems:"center",justifyContent:"space-between",fontSize:12,color:"#b0b0d0"}}>
            <span style={{display:"flex",alignItems:"center",gap:5}}>
              Live sync
              {syncPending&&<span style={{fontSize:9,color:"#fc9c4c",letterSpacing:"0.05em"}}>↻ syncing…</span>}
            </span>
            <div onClick={()=>setLiveSync(v=>!v)} style={{position:"relative",width:32,height:18,borderRadius:9,
              background:liveSync?"#7c5cfc":"#22223a",cursor:"pointer",transition:"background .15s",flexShrink:0}}>
              <div style={{position:"absolute",width:12,height:12,borderRadius:"50%",background:"#07070e",
                top:3,left:liveSync?17:3,transition:"left .15s"}}/>
            </div>
          </div>
        </div>
        {!blocks.length&&(
          <div style={{fontFamily:"monospace",fontSize:9,color:MUT,textAlign:"center"}}>
            add blocks to the sequence first
          </div>
        )}
        <div style={{display:"flex",gap:5}}>
          <Btn v="g" onClick={blocks.length&&!piRunning ? runOnPi : undefined}
            style={{flex:2,opacity:(!blocks.length||piRunning)?0.35:1,
              cursor:(!blocks.length||piRunning)?"not-allowed":"pointer"}}>
            {piRunning?"sending…":"▶ Run on Pi4"}
          </Btn>
          <Btn v="r" onClick={stopMotor} style={{flex:1,fontWeight:"bold",fontSize:13}}>
            ⬛ STOP
          </Btn>
        </div>

        {piErr&&<div style={{fontFamily:"monospace",fontSize:10,color:"#fc5c7c",
          padding:"4px 6px",background:"#1e0808",borderRadius:4,
          wordBreak:"break-all"}}>{piErr}</div>}
      </div>

      {/* ── Right: visualizer ──────────────────────────── */}
      <div style={{flex:1,display:"grid",gridTemplateRows:"1fr 110px",
        overflow:"hidden",padding:8,gap:7,minWidth:0}}>
        <ArmCanvas deg={animDeg} trail={trail}/>
        <TLCanvas  sim={sim}     marker={marker}/>
      </div>

    </div>
  );
}
