#include "WebContent.h"

namespace Hexapod {

const char kIndexHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Hexapod Control</title>
  <style>
    :root{color-scheme:dark;--bg:#101315;--panel:#1b2226;--line:#35434a;--text:#f3f7f8;--muted:#9facb2;--green:#3bd493;--amber:#ffb648;--red:#f06b6b}
    *{box-sizing:border-box}
    [hidden]{display:none!important}
    body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,sans-serif}
    main{width:min(720px,100%);margin:auto;padding:22px}
    h1{margin:0 0 6px}
    .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
    .tof-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
    .card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px}
    .card span{display:block;color:var(--muted);font-size:13px}
    .card strong{display:block;margin-top:5px;font-size:23px}
    .receiving{color:var(--green)}.stale{color:var(--red)}
    .controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:18px 0}
    .tracking-controls{grid-template-columns:1fr 1fr;margin-bottom:10px}
    .stand-controls{grid-template-columns:repeat(2,1fr);margin:10px 0}
    .charge-controls{grid-template-columns:repeat(3,1fr);margin-top:0}
    .emergency-controls{grid-template-columns:1fr 1fr;margin:10px 0}
    button.full{width:100%}
    .manual-result{min-height:20px;margin:8px 0 0;color:var(--muted)}
    .sensor-detail{display:block;margin-top:6px;color:var(--muted);font-size:12px;line-height:1.45}
    .manual-result.good{color:var(--green)}.manual-result.bad{color:var(--red)}
    button{min-height:48px;border:1px solid var(--line);border-radius:10px;color:var(--text);font:inherit;font-weight:700;cursor:pointer}
    button:disabled{cursor:not-allowed;opacity:.45}
    .start{background:#207253}.stand{background:#7b5c1d}.stop{background:#74363a}.reset{background:#355163}.tilt{background:#5c477c}.charge{background:#386478}.fire{background:#8a4d19}
    h2{margin:20px 0 8px;font-size:18px}h3{margin:0 0 8px;font-size:16px}
    a{color:#86c7ff}
    footer{margin-top:18px;color:var(--muted);font-size:13px}
    @media(max-width:520px){.grid,.tof-grid,.controls{grid-template-columns:1fr}}
  </style>
</head>
<body>
<main>
  <h1>Hexapod Control</h1>
  <h2>System status</h2>
  <section class="grid">
    <div class="card"><span>Connection</span><strong id="connection">Waiting...</strong></div>
    <div class="card"><span>Servo boards</span><strong id="servo-boards">Checking...</strong></div>
    <div class="card"><span>Robot state</span><strong id="robot-state">IDLE</strong></div>
    <div class="card"><span>Aim</span><strong id="aim-state">--</strong></div>
    <div class="card"><span>Target distance</span><strong id="camera-distance">--</strong></div>
    <div class="card"><span>Charge</span><strong id="charge-state">READY</strong></div>
  </section>
  <h2>Distance sensors</h2>
  <section class="card">
    <div class="tof-grid">
      <div class="card">
        <span>Right</span>
        <strong id="tof-ch0-distance">--</strong>
        <small id="tof-ch0-detail" class="sensor-detail">Waiting for sensor...</small>
      </div>
      <div class="card">
        <span>Front</span>
        <strong id="tof-ch1-distance">--</strong>
        <small id="tof-ch1-detail" class="sensor-detail">Waiting for sensor...</small>
      </div>
      <div class="card">
        <span>Left</span>
        <strong id="tof-ch7-distance">--</strong>
        <small id="tof-ch7-detail" class="sensor-detail">Waiting for sensor...</small>
      </div>
    </div>
    <p id="tof-fault" class="manual-result" hidden></p>
    <div id="obstacle-nav-card" class="card" hidden>
      <span>Obstacle navigation</span>
      <strong id="obstacle-nav-state"></strong>
      <small id="obstacle-nav-reason" class="sensor-detail"></small>
    </div>
  </section>
  <h2>Movement</h2>
  <section class="card">
    <h3>Setup</h3>
    <button id="standup" class="stand full" onclick="startStandup()">Stand up</button>
  </section>
  <section class="card">
    <h3>Walking</h3>
    <div class="controls stand-controls">
      <button id="wave-start" class="start" onclick="startWaveCycle()">One-leg wave walk</button>
      <button id="dual-start" class="stand" onclick="startDualRipple()">Dual-ripple walk with push</button>
      <button id="continuous-ripple-v4-start" class="stand" onclick="startContinuousRipple('v4')">V4 ripple walk</button>
      <button id="continuous-ripple-v6-start" class="stand" onclick="startContinuousRipple('v6')">V6 ripple walk</button>
      <button id="obstacle-v6-start" class="start" onclick="startContinuousRipple('obstacle')">V6 obstacle avoidance</button>
    </div>
    <div class="controls emergency-controls">
      <button id="wave-stop" class="stop" onclick="stopWaveWalk()">Stop walking</button>
      <button id="legs-off" class="stop" onclick="disableLegPwm()">Turn leg servos off</button>
    </div>
  </section>
  <section class="card">
    <h3>Turning</h3>
    <div class="controls tracking-controls">
      <button id="turn-cw-180-fast" class="start" onclick="startFastTurn180('cw')">Fast 180&deg; clockwise</button>
      <button id="turn-ccw-180-fast" class="start" onclick="startFastTurn180('ccw')">Fast 180&deg; counterclockwise</button>
    </div>
    <h3>Calculated diagonal paths</h3>
    <div class="controls tracking-controls">
      <button id="path-forward-left" class="stand" onclick="startCalculatedPath('left')">30 cm forward + 30 cm left</button>
      <button id="path-forward-right" class="stand" onclick="startCalculatedPath('right')">30 cm forward + 30 cm right</button>
    </div>
  </section>
  <p id="stand-result" class="manual-result"></p>
  <h2>Automatic cannon setup</h2>
  <section class="card">
    <div class="controls tracking-controls">
      <button id="cannon-prepare-auto" class="start" onclick="startAutomaticCannonPreparation()">Full automatic setup</button>
      <button id="cannon-prepare-start" class="start" onclick="setCannonPreparation(true)">Aim and calculate</button>
      <button id="cannon-prepare-confirm" class="start" onclick="confirmCannonPreparation()">Confirm plan</button>
      <button id="cannon-prepare-confirm-tilt-home" class="start" onclick="confirmTiltFullyDown()">Confirm tilt down</button>
      <button id="cannon-prepare-confirm-tilt-up" class="charge" onclick="confirmTiltUp()">Confirm tilt and charge</button>
      <button id="cannon-prepare-add-charge" class="charge" onclick="addPreparationChargeStep()">Add 200 ms pull</button>
      <button id="cannon-prepare-confirm-unwind" class="reset" onclick="confirmChargedAndUnwind()">Unwind charge</button>
      <button id="cannon-prepare-cancel" class="stop" onclick="setCannonPreparation(false)">Cancel setup</button>
    </div>
    <strong id="cannon-prepare-state" hidden></strong>
    <p id="cannon-prepare-plan" class="manual-result" hidden></p>
    <p id="cannon-prepare-result" class="manual-result" hidden></p>
  </section>
  <h2>Manual cannon controls</h2>
  <section class="card">
    <h3>Aiming</h3>
    <div class="controls tracking-controls">
      <button id="start" class="start" onclick="setServo(true)">Start tracking</button>
      <button id="stop" class="stop" onclick="setServo(false)">Stop tracking</button>
      <button id="cannon-left" class="tilt" onclick="nudgeCannon('left')">Turn left (250 ms)</button>
      <button id="cannon-right" class="tilt" onclick="nudgeCannon('right')">Turn right (250 ms)</button>
    </div>
    <p id="aim-manual-result" class="manual-result"></p>
  </section>
  <section class="card">
    <h3>Tilt</h3>
    <div class="controls tracking-controls">
      <button id="tilt-up" class="tilt" onclick="triggerTilt('up')">Tilt up (200 ms)</button>
      <button id="tilt-down" class="tilt" onclick="triggerTilt('down')">Tilt down (200 ms)</button>
    </div>
    <p id="tilt-result" class="manual-result"></p>
  </section>
  <section class="card">
    <h3>Charge</h3>
    <div class="controls charge-controls">
      <button id="charge-p2" class="charge" onclick="triggerCharge('p2')">Charge P2 (6000 ms)</button>
      <button id="charge-half" class="charge" onclick="triggerCharge('charge-half')">Charge +500 ms</button>
      <button id="uncharge-half" class="reset" onclick="triggerCharge('uncharge-half')">Release charge (500 ms)</button>
      <button id="uncharge-recorded" class="reset" onclick="triggerCharge('unwind-recorded')">Unwind charge</button>
      <button id="charge-home" class="reset" onclick="triggerCharge('mark-home')">Confirm charge is at HOME</button>
      <button id="charge-stop" class="stop" onclick="triggerCharge('stop')">Stop charge</button>
    </div>
    <p id="charge-result" class="manual-result"></p>
  </section>
  <h2>Fire</h2>
  <section class="card">
    <button id="shoot" class="fire full" onclick="triggerShoot()">Fire</button>
    <p id="fire-result" class="manual-result"></p>
  </section>
  <footer><a href="/update">Firmware update</a></footer>
</main>
<script>
async function sendCommand(url,{resultId,buttonIds=[],successText=null}={}){
  const result=resultId?document.getElementById(resultId):null;
  buttonIds.forEach(id=>document.getElementById(id).disabled=true);
  try{
    const response=await fetch(url,{method:'POST'});
    const message=await response.text();
    if(!response.ok)throw new Error(message||('Request failed: '+response.status));
    if(result){
      result.textContent=successText===null?message:successText;
      result.className='manual-result good';
    }
    return true;
  }catch(error){
    if(result){
      result.textContent=error.message;
      result.className='manual-result bad';
    }
    return false;
  }finally{
    await refresh();
  }
}
function nudgeCannon(direction){
  return sendCommand('/api/servo/nudge/'+direction,{
    resultId:'aim-manual-result',buttonIds:['cannon-left','cannon-right']
  });
}
function startStandup(){
  return sendCommand('/api/standup',{
    resultId:'stand-result',buttonIds:['standup']
  });
}
function startWaveCycle(){
  if(!confirm('Start continuous wave walking?'))return;
  return sendCommand('/api/legs/wave-cycle',{
    resultId:'stand-result',buttonIds:['wave-start']
  });
}
function startDualRipple(){
  if(!confirm('Start dual ripple gait?'))return;
  return sendCommand('/api/legs/dual-ripple',{
    resultId:'stand-result',buttonIds:['dual-start']
  });
}
function startContinuousRipple(version){
  const obstacle=version==='obstacle';
  const version6=version==='v6';
  const buttonId=obstacle?'obstacle-v6-start':version6?'continuous-ripple-v6-start':'continuous-ripple-v4-start';
  const description=obstacle?'V6 obstacle avoidance':
    version6?'V6 ripple gait':'V4 ripple gait';
  if(!confirm('Start '+description+'?'))return;
  const endpoint=obstacle?'/api/legs/obstacle-v6':version6?'/api/legs/continuous-ripple-v6':'/api/legs/continuous-ripple-v4';
  return sendCommand(endpoint,{resultId:'stand-result',buttonIds:[buttonId]});
}
function startFastTurn180(direction){
  const label=direction==='cw'?'clockwise':'counterclockwise';
  if(!confirm('Start fast 180° '+label+' turn?'))return;
  return sendCommand('/api/legs/turn-180-fast/'+direction,{
    resultId:'stand-result',buttonIds:['turn-'+direction+'-180-fast']
  });
}
function startCalculatedPath(side){
  if(!confirm('Start calculated path: 30 cm forward and 30 cm '+side+'?'))return;
  return sendCommand('/api/legs/path-forward-'+side,{
    resultId:'stand-result',buttonIds:['path-forward-'+side]
  });
}
function stopWaveWalk(){
  return sendCommand('/api/legs/wave-stop',{
    resultId:'stand-result',buttonIds:['wave-stop']
  });
}
function disableLegPwm(){
  return sendCommand('/api/legs/off',{
    resultId:'stand-result',buttonIds:['legs-off']
  });
}
function startAutomaticCannonPreparation(){
  if(!confirm('Start FULL automatic cannon setup?'))return;
  return sendCommand('/api/cannon/prepare/auto',{
    resultId:'cannon-prepare-result',
    buttonIds:['cannon-prepare-auto','cannon-prepare-start','cannon-prepare-cancel']
  });
}
function setCannonPreparation(enabled){
  if(enabled&&!confirm('Start cannon aim and calculation?'))return;
  return sendCommand(enabled?'/api/cannon/prepare/start':'/api/cannon/prepare/cancel',{
    resultId:'cannon-prepare-result',
    buttonIds:['cannon-prepare-auto','cannon-prepare-start','cannon-prepare-cancel']
  });
}
async function confirmCannonPreparation(){
  const result=document.getElementById('cannon-prepare-result');
  try{
    const statusResponse=await fetch('/status',{cache:'no-store'});
    if(!statusResponse.ok)throw new Error('Status HTTP '+statusResponse.status);
    const data=await statusResponse.json();
    if(!data.cannonPreparationAwaitingConfirmation)throw new Error('No calculated plan is awaiting confirmation.');
    const correction=data.cannonPreparationChargeCorrectionMs>=0
      ?'+'+data.cannonPreparationChargeCorrectionMs:String(data.cannonPreparationChargeCorrectionMs);
    const plan='Distance '+data.cannonPreparationDistanceCm.toFixed(1)+' cm\n'+
      'Tilt up '+data.cannonPreparationTiltUpMs+' ms\n'+
      'Power P'+data.cannonPreparationPowerLevel+'\n'+
      'Pull '+data.cannonPreparationPlannedChargeDurationMs+' ms (correction '+correction+' ms)';
    if(!confirm(plan+'\n\nConfirm plan?'))return;
  }catch(error){
    result.textContent=error.message;
    result.className='manual-result bad';
    return;
  }
  return sendCommand('/api/cannon/prepare/confirm',{
    resultId:'cannon-prepare-result',buttonIds:['cannon-prepare-confirm']
  });
}
function confirmTiltFullyDown(){
  if(!confirm('Confirm cannon is fully down?'))return;
  return sendCommand('/api/cannon/prepare/tilt-home/confirm',{
    resultId:'cannon-prepare-result',buttonIds:['cannon-prepare-confirm-tilt-home']
  });
}
function confirmTiltUp(){
  if(!confirm('Is the barrel angle correct?'))return;
  return sendCommand('/api/cannon/prepare/tilt-up/confirm',{
    resultId:'cannon-prepare-result',buttonIds:['cannon-prepare-confirm-tilt-up']
  });
}
function addPreparationChargeStep(){
  if(!confirm('Add 200 ms pull?'))return;
  return sendCommand('/api/cannon/prepare/charge/add-200ms',{
    resultId:'cannon-prepare-result',buttonIds:['cannon-prepare-add-charge']
  });
}
function confirmChargedAndUnwind(){
  if(!confirm('Confirm charge and unwind?'))return;
  return sendCommand('/api/cannon/prepare/confirm-charged-unwind',{
    resultId:'cannon-prepare-result',buttonIds:['cannon-prepare-confirm-unwind']
  });
}
function setServo(enabled){
  return sendCommand(enabled?'/api/servo/start':'/api/servo/stop',{
    resultId:'aim-manual-result',buttonIds:['start','stop'],
    successText:enabled?'Tracking started.':'Tracking stopped.'
  });
}
function triggerShoot(){
  if(!confirm('Fire now?'))return;
  return sendCommand('/api/shoot',{
    resultId:'fire-result',buttonIds:['shoot']
  });
}
function triggerTilt(direction){
  return sendCommand('/api/tilt/'+direction,{
    resultId:'tilt-result',buttonIds:['tilt-up','tilt-down']
  });
}
function triggerCharge(action){
  if(action==='mark-home'&&!confirm('Set charge HOME?'))return;
  return sendCommand('/api/charge/'+action,{
    resultId:'charge-result',
    buttonIds:['charge-half','charge-p2','uncharge-recorded','uncharge-half','charge-home','charge-stop']
  });
}
function updateTofChannel(sensor,channel){
  const distance=document.getElementById('tof-ch'+channel+'-distance');
  const detail=document.getElementById('tof-ch'+channel+'-detail');
  if(!sensor){
    distance.textContent='--';
    distance.className='stale';
    detail.hidden=false;
    detail.textContent='CH'+channel+' · no sensor status';
    return;
  }
  distance.textContent=sensor.valid?sensor.filteredMm+' mm':'--';
  distance.className=sensor.valid?'receiving':'stale';
  if(sensor.valid){
    detail.hidden=true;
    detail.textContent='';
    return;
  }
  detail.hidden=false;
  const raw='raw '+sensor.rawMm+' mm';
  const age=sensor.ageMs===null?'no sample yet':'age '+sensor.ageMs+' ms';
  const state=!sensor.initialized?'NOT INITIALIZED':sensor.timedOut?'TIMEOUT / STALE':'INVALID RANGE';
  detail.textContent='CH'+channel+' · '+state+' · '+raw+' · '+age;
}
async function refresh(){
  try{
    const statusResponse=await fetch('/status',{cache:'no-store'});
    if(!statusResponse.ok)throw new Error('Status HTTP '+statusResponse.status);
    const data=await statusResponse.json();
    const connection=document.getElementById('connection');
    connection.textContent=!data.hasData?'Waiting':!data.aimTagsVisible
      ?'TAG LOST ('+(data.cannonTagDetected?'cannon ':'')+(data.targetTagDetected?'target':'')+')'
      :data.controlFresh?'Receiving':data.dataRecent?'Too old for motion':'Stale';
    connection.className=data.controlFresh?'receiving':(data.hasData?'stale':'');
    const servoBoards=document.getElementById('servo-boards');
    servoBoards.textContent='0x40 '+(data.pcaReady?'ONLINE':'OFFLINE')+
      ' · 0x41 '+(data.pca41Ready?'ONLINE':'OFFLINE');
    servoBoards.className=data.pcaReady&&data.pca41Ready?'receiving':'stale';
    const tof=data.tof||{started:false,fault:'No ToF status received',sensors:[]};
    [0,1,7].forEach(channel=>{
      updateTofChannel(tof.sensors.find(sensor=>sensor.channel===channel),channel);
    });
    const tofFault=document.getElementById('tof-fault');
    if(!tof.started){
      tofFault.hidden=false;
      tofFault.textContent='Diagnostics unavailable: '+(tof.fault||'mux or sensors not detected');
      tofFault.className='manual-result bad';
    }else if(tof.fault){
      tofFault.hidden=false;
      tofFault.textContent='Partial sensor warning: '+tof.fault;
      tofFault.className='manual-result bad';
    }else{
      tofFault.hidden=true;
      tofFault.textContent='';
      tofFault.className='manual-result';
    }
    const obstacleCard=document.getElementById('obstacle-nav-card');
    const obstacleState=document.getElementById('obstacle-nav-state');
    const obstacleStateText=data.obstacleNavigationState||'idle';
    obstacleCard.hidden=obstacleStateText==='idle'&&!data.obstacleNavigationActive;
    obstacleState.textContent=obstacleStateText.toUpperCase();
    obstacleState.className=(obstacleStateText==='blocked'||obstacleStateText==='sensor fault')
      ?'stale':data.obstacleNavigationActive?'receiving':'';
    document.getElementById('obstacle-nav-reason').textContent=
      data.obstacleNavigationReason||'';
    const cannonPrepareState=document.getElementById('cannon-prepare-state');
    const cannonPrepareStateText=data.cannonPreparationState||'idle';
    cannonPrepareState.hidden=cannonPrepareStateText==='idle';
    cannonPrepareState.textContent=cannonPrepareStateText.toUpperCase();
    cannonPrepareState.className=data.cannonPreparationReady
      ?'receiving':cannonPrepareStateText==='fault'?'stale':'';
    const cannonPrepareResult=document.getElementById('cannon-prepare-result');
    cannonPrepareResult.hidden=cannonPrepareStateText==='idle';
    cannonPrepareResult.textContent=cannonPrepareStateText==='idle'
      ?'':data.cannonPreparationReason||'';
    const planNode=document.getElementById('cannon-prepare-plan');
    if(data.cannonPreparationPowerLevel>0){
      planNode.hidden=false;
      const correction=data.cannonPreparationChargeCorrectionMs>=0
        ?'+'+data.cannonPreparationChargeCorrectionMs
        :String(data.cannonPreparationChargeCorrectionMs);
      planNode.textContent='PLAN | distance '+data.cannonPreparationDistanceCm.toFixed(1)+' cm | tilt up '+
        data.cannonPreparationTiltUpMs+' ms | P'+data.cannonPreparationPowerLevel+
        ' nominal '+data.cannonPreparationBaseChargeDurationMs+' ms | effective pull '+
        data.cannonPreparationPlannedChargeDurationMs+' ms | correction '+correction+' ms';
      planNode.className=data.cannonPreparationAwaitingConfirmation?'manual-result good':'manual-result';
    }else{
      planNode.hidden=true;
      planNode.textContent='';
      planNode.className='manual-result';
    }
    document.getElementById('standup').disabled=
      data.standActive||!data.pcaReady||!data.pca41Ready;
    document.getElementById('wave-start').disabled=
      data.standActive||!data.waveCanStart||!data.pcaReady||!data.pca41Ready;
    document.getElementById('dual-start').disabled=
      data.standActive||!data.dualCanStart||!data.pcaReady||!data.pca41Ready;
    document.getElementById('continuous-ripple-v4-start').disabled=
      data.standActive||!data.continuousRippleCanStart||!data.pcaReady||!data.pca41Ready;
    document.getElementById('continuous-ripple-v6-start').disabled=
      data.standActive||!data.continuousRippleCanStart||!data.pcaReady||!data.pca41Ready;
    document.getElementById('obstacle-v6-start').disabled=
      data.standActive||!data.continuousRippleCanStart||!data.obstacleNavigationCanStart||!data.pcaReady||!data.pca41Ready;
    ['turn-cw-180-fast','turn-ccw-180-fast','path-forward-left','path-forward-right'].forEach(id=>{
      document.getElementById(id).disabled=data.standActive||!data.turnCanStart||!data.pcaReady||!data.pca41Ready;
    });
    document.getElementById('wave-stop').disabled=!data.waveCanStop;
    document.getElementById('legs-off').disabled=!data.pcaReady&&!data.pca41Ready;
    document.getElementById('cannon-prepare-auto').disabled=
      !data.cannonPreparationCanStart||!data.pcaReady;
    document.getElementById('cannon-prepare-start').disabled=
      !data.cannonPreparationCanStart||!data.pcaReady;
    document.getElementById('cannon-prepare-confirm').disabled=
      !data.cannonPreparationAwaitingConfirmation||!data.pcaReady;
    document.getElementById('cannon-prepare-confirm-tilt-home').disabled=
      !data.cannonPreparationAwaitingTiltHomeConfirmation||data.tiltActive||!data.pcaReady;
    document.getElementById('cannon-prepare-confirm-tilt-up').disabled=
      !data.cannonPreparationAwaitingTiltUpConfirmation||data.tiltActive||data.chargeActive||!data.pcaReady;
    document.getElementById('cannon-prepare-add-charge').disabled=
      !data.cannonPreparationCanAddChargeStep||!data.pcaReady;
    const confirmUnwind=document.getElementById('cannon-prepare-confirm-unwind');
    confirmUnwind.disabled=
      !data.cannonPreparationAwaitingUnwindConfirmation||!data.pcaReady;
    confirmUnwind.textContent=data.cannonPreparationAwaitingUnwindConfirmation
      ?'Unwind charge ('+(data.estimatedChargeWoundDurationMs+500)+' ms)'
      :'Unwind charge';
    document.getElementById('cannon-prepare-cancel').disabled=
      !data.cannonPreparationActive;
    document.getElementById('start').disabled=
      data.cannonPreparationActive||data.standActive||data.servoEnabled||!data.pcaReady;
    document.getElementById('stop').disabled=
      data.standActive||(!data.servoEnabled&&!data.manualAimActive&&!data.cannonPreparationActive);
    ['cannon-left','cannon-right'].forEach(id=>{
      document.getElementById(id).disabled=
        data.cannonPreparationActive||data.standActive||data.manualAimActive||!data.pcaReady;
    });
    document.getElementById('shoot').disabled=
      data.standActive||data.shootActive||!data.pcaReady||
      (data.cannonPreparationActive&&!data.cannonPreparationReady);
    ['tilt-up','tilt-down'].forEach(id=>{
      document.getElementById(id).disabled=
        data.cannonPreparationActive||data.standActive||data.tiltActive||!data.pcaReady;
    });
    document.getElementById('charge-half').disabled=
      data.cannonPreparationActive||data.standActive||data.chargeActive||!data.pcaReady;
    document.getElementById('charge-p2').disabled=
      data.cannonPreparationActive||data.standActive||data.chargeActive||
      !data.chargeAtEstimatedHome||!data.pcaReady;
    document.getElementById('uncharge-recorded').disabled=
      data.cannonPreparationActive||data.standActive||data.chargeActive||
      data.estimatedChargeWoundDurationMs===0||!data.pcaReady;
    document.getElementById('uncharge-recorded').textContent=
      data.estimatedChargeWoundDurationMs>0
        ?'Unwind charge ('+(data.estimatedChargeWoundDurationMs+500)+' ms)'
        :'Unwind charge';
    document.getElementById('uncharge-half').disabled=
      data.cannonPreparationActive||data.standActive||data.chargeActive||!data.pcaReady;
    document.getElementById('charge-home').disabled=
      data.cannonPreparationActive||data.standActive||data.chargeActive;
    document.getElementById('charge-stop').disabled=
      !data.chargeActive&&!data.cannonPreparationActive;
    document.getElementById('charge-state').textContent=
      data.chargeActive
        ? data.chargeAction.toUpperCase()+' ('+data.chargeRemainingMs+' ms)'
        : data.chargeAtEstimatedHome
        ? 'HOME (software estimate)'
        : 'WOUND '+data.estimatedChargeWoundDurationMs+' ms';
    const robotState=document.getElementById('robot-state');
    robotState.textContent=
      (data.standMode==='none'?'':data.standMode.toUpperCase()+' · ')+
      data.standStage.toUpperCase()+
      (data.standActive?' ('+data.standProgressPct.toFixed(0)+'%)':'');
    robotState.className=data.standFault?'stale':data.standActive?'receiving':'';
    const aimState=document.getElementById('aim-state');
    aimState.textContent=(data.servoEnabled?'TRACKING ON':'TRACKING OFF')+' · '+
      (data.aimTagsVisible
        ?data.direction.toUpperCase()+' · '+data.errorDeg.toFixed(2)+'°'
        :'TAG LOST');
    aimState.className=!data.controlFresh?'stale':data.servoEnabled?'receiving':'';
    if(data.standFault){
      const standResult=document.getElementById('stand-result');
      standResult.textContent=data.standFault;
      standResult.className='manual-result bad';
    }
    document.getElementById('camera-distance').textContent=
      data.aimTagsVisible&&data.cameraTargetDistanceCm!==null
        ?data.cameraTargetDistanceCm.toFixed(1)+' cm':'--';
  }catch(error){
    const connection=document.getElementById('connection');
    connection.textContent='Disconnected';
    connection.className='stale';
  }
}
setInterval(refresh,250);
refresh();
</script>
</body>
</html>
)rawliteral";

const char kUpdateHtml[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>AprilTag Receiver OTA</title>
  <style>
    body{max-width:620px;margin:40px auto;padding:0 16px;font-family:system-ui,sans-serif}
    form{display:grid;gap:14px;padding:20px;border:1px solid #aaa;border-radius:10px}
    button,input{font:inherit;padding:10px}
    .download{display:block;margin-top:14px;padding:12px;text-align:center;border:1px solid #aaa;border-radius:10px}
  </style>
</head>
<body>
  <h1>OTA firmware update</h1>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="firmware" accept=".bin" required>
    <button type="submit">Upload firmware.bin</button>
  </form>
  <a class="download" href="/firmware.bin">Download current firmware.bin</a>
  <p><a href="/">Back to receiver</a></p>
</body>
</html>
)rawliteral";


} // namespace Hexapod
