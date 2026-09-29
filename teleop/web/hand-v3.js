'use strict';
// Read both primary and secondary native sources. Controllers may remain primary.
function handSources(xrSession,side){
 const seen=new Set(),out=[];
 for(const source of ['inputSources','trackedSources'])for(const input of xrSession[source]??[]){
  if(seen.has(input))continue;seen.add(input);
  if(input.handedness===side&&input.hand)out.push({input,source});
 }
 return out;
}
function readNativeHand(candidates,frame,referenceSpace,jointNames){
 let missing=null;
 for(const candidate of candidates){
  const points=[];let wrist=null,valid=true;
  for(const name of jointNames){
   const jointSpace=candidate.input.hand.get(name),joint=jointSpace&&frame.getJointPose(jointSpace,referenceSpace);
   if(!joint||joint.emulatedPosition){missing??=name;valid=false;break;}
   const p=joint.transform.position;
   if(![p.x,p.y,p.z].every(Number.isFinite)){missing??=name;valid=false;break;}
   points.push([p.x,p.y,p.z]);if(name==='wrist')wrist=joint.transform.orientation;
  }
  if(valid&&wrist&&[wrist.x,wrist.y,wrist.z,wrist.w].every(Number.isFinite))return {valid:true,points,wrist,source:candidate.source};
 }
 return {valid:false,missing};
}
function report(phase,error){fetch('/client_event',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({version:3,phase,error})}).catch(()=>{});}
const $=id=>document.getElementById(id),names=['wrist'];
for(const finger of ['thumb','index-finger','middle-finger','ring-finger','pinky-finger']){
 const parts=finger==='thumb'?['metacarpal','phalanx-proximal','phalanx-distal','tip']:['phalanx-proximal','phalanx-intermediate','phalanx-distal','tip'];
 names.push(...parts.map(p=>finger+'-'+p));
}
const edges=[[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],[13,17],[0,17],[17,18],[18,19],[19,20]];
const startup={};
function mark(name){if(started&&!Object.hasOwn(startup,name)){startup[name]=performance.now()-started;report(name,JSON.stringify(startup));if(ready)ws.send(JSON.stringify({type:'metrics',...startup}));}}
let ws=null,session=null,space=null,challenge=null,ready=false,seq=0,wantsFollow=true,entering=false,validBefore=false,mode='immersive-vr',started=0,firstFrame=false,reconnectTimer=null,lastSourceSummary='';
function describeSources(){
 const describe=list=>Array.from(list??[],x=>`${x.handedness}:${x.hand?'hand':'controller'}`);
 const summary=JSON.stringify({input:describe(session.inputSources),tracked:describe(session.trackedSources),api:'trackedSources' in session,features:Array.from(session.enabledFeatures??[])});
 if(summary!==lastSourceSummary){lastSourceSummary=summary;report('native_sources',summary);}
}
if(navigator.xr)navigator.xr.isSessionSupported('immersive-ar').then(ok=>{if(ok)mode='immersive-ar';}).catch(()=>{});
function command(type){if(ws?.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type}));}
let claimSent=false;
function claim(){
 if(session&&ws?.readyState===WebSocket.OPEN&&!claimSent){claimSent=true;ws.send(JSON.stringify({type:'start',enable:wantsFollow}));}
}
function connect(){
 if(ws&&(ws.readyState===WebSocket.OPEN||ws.readyState===WebSocket.CONNECTING))return;
 mark('ws_connect_ms');
 const socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws?v=3`);ws=socket;ready=false;challenge=null;claimSent=false;
 socket.onopen=()=>{mark('ws_open_ms');claim();};
 socket.onmessage=e=>{if(socket!==ws)return;const m=JSON.parse(e.data);if(m.type==='challenge'){challenge=m.id;mark('challenge_ms');}if(m.type==='ready'){ready=true;mark('ws_ready_ms');}if(m.type==='released'){ready=false;claimSent=false;}};
 socket.onclose=e=>{if(socket!==ws)return;ready=false;challenge=null;claimSent=false;
  if(e.code===4001){$('status').textContent='已在另一个页面开始跟踪';if(session)session.end();return;}
  if(session||document.visibilityState==='visible')reconnectTimer=setTimeout(connect,200);
 };
}
connect();
document.addEventListener('visibilitychange',()=>{if(document.visibilityState==='visible')connect();});
async function setFollow(value){
 wantsFollow=value;
 if(ready)command(value?'enable':'pause');
 else await fetch('/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({version:3,action:value?'enable':'pause'})});
 $('status').textContent=value?'已启用，等待左手':'已手动暂停';
}
$('pause').onclick=()=>setFollow(false);$('resume').onclick=()=>setFollow(true);
async function poll(){try{
 const r=await fetch('/state',{cache:'no-store'}),s=await r.json();
 $('status').textContent=s.status;
 const h=s.hardware,bridge=h?.last?.bridge_ms;
 $('stats').textContent=`映射 ${s.retarget_ms?.toFixed(1)??'—'} ms · 驱动往返 ${bridge?.toFixed(1)??'—'} ms · 已发送 ${h?.sent??0}`;
 if($('debug').parentNode.open)$('debug').textContent=JSON.stringify(s,null,2);
 }catch{$('status').textContent='服务连接中…';}finally{setTimeout(poll,750);}}
poll();
report('page_ready','native-sources-1');
const canvas=$('canvas'),gl=canvas.getContext('webgl',{xrCompatible:true,alpha:true,antialias:false,powerPreference:'high-performance'});
function shader(kind,source){const s=gl.createShader(kind);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s;}
const program=gl.createProgram();
gl.attachShader(program,shader(gl.VERTEX_SHADER,'attribute vec3 p;uniform mat4 proj;uniform mat4 view;void main(){gl_Position=proj*view*vec4(p,1.);gl_PointSize=6.;}'));
gl.attachShader(program,shader(gl.FRAGMENT_SHADER,'precision mediump float;void main(){gl_FragColor=vec4(.2,1.,.55,1.);}'));gl.linkProgram(program);
const buffer=gl.createBuffer(),pos=gl.getAttribLocation(program,'p'),proj=gl.getUniformLocation(program,'proj'),view=gl.getUniformLocation(program,'view'),vertices=new Float32Array(edges.length*6);
function draw(pose,points){
 const layer=session.renderState.baseLayer;gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);gl.clearColor(0,0,0,0);gl.clear(gl.COLOR_BUFFER_BIT);
 if(!pose||!points)return;
 let i=0;for(const [a,b] of edges)for(const j of [a,b])for(let k=0;k<3;k++)vertices[i++]=points[j][k];
 gl.useProgram(program);gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,vertices,gl.DYNAMIC_DRAW);gl.enableVertexAttribArray(pos);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,0,0);
 for(const v of pose.views){const vp=layer.getViewport(v);gl.viewport(vp.x,vp.y,vp.width,vp.height);gl.uniformMatrix4fv(proj,false,v.projectionMatrix);gl.uniformMatrix4fv(view,false,v.transform.inverse.matrix);gl.drawArrays(gl.LINES,0,vertices.length/3);}
}
function lost(reason){if(validBefore&&ready)ws.send(JSON.stringify({type:'lost',reason}));validBefore=false;}
function tick(t,frame){
 if(!session||frame.session!==session)return;session.requestAnimationFrame(tick);
 try{
 mark('first_xr_frame_ms');
 const pose=frame.getViewerPose(space),candidates=handSources(session,'left');
 if(!candidates.length&&Array.from(session.inputSources).some(x=>!x.hand))mark('controller_input_ms');
 if(candidates.length)mark('hand_input_ms');if(pose)mark('viewer_pose_ms');if(session.visibilityState==='visible')mark('visible_ms');
 const hand=pose&&session.visibilityState==='visible'?readNativeHand(candidates,frame,space,names):{valid:false};
 const {points,wrist}=hand,valid=hand.valid;
 if(hand.missing)mark('waiting_joint_'+hand.missing+'_ms');
 if(valid)mark('all_joints_ms');
 if(valid)mark(hand.source==='trackedSources'?'tracked_hand_ms':'primary_hand_ms');
 const trackingMessage=valid?'左手骨架已就绪':session.visibilityState!=='visible'?'头显未处于跟踪画面':!pose?'等待头显定位':candidates.length?'等待左手关节定位':'等待 Quest 提供左手骨架';
 if($('tracking').textContent!==trackingMessage)$('tracking').textContent=trackingMessage;
 if(valid&&ready&&challenge&&ws.bufferedAmount===0){
  const q=pose.transform.orientation;
  ws.send(JSON.stringify({type:'frame',seq:++seq,challenge,valid:true,side:'left',points,wrist_q:[wrist.x,wrist.y,wrist.z,wrist.w],head_q:[q.x,q.y,q.z,q.w]}));
  validBefore=true;
  if(!firstFrame){firstFrame=true;mark('first_frame_ms');report('native_first_hand',JSON.stringify({source:hand.source,first_frame_ms:startup.first_frame_ms,hand_input_ms:startup.hand_input_ms,xr_ready_ms:startup.xr_ready_ms}));}
 }else if(!valid)lost('等待左手进入视野');
 draw(pose,valid?points:null);
 }catch(e){lost('跟踪暂时不可用');$('status').textContent=e.message;}
}
$('start').onclick=async()=>{
 if(entering||session)return;entering=true;for(const key of Object.keys(startup))delete startup[key];started=performance.now();firstFrame=false;lastSourceSummary='';wantsFollow=true;$('start').disabled=true;$('status').textContent='正在进入裸手跟踪…';
 try{
  report('xr_requested');
  if(!navigator.xr)throw Error('请在 Quest 浏览器中打开 localhost:8042');
  session=await navigator.xr.requestSession(mode,{requiredFeatures:['hand-tracking'],optionalFeatures:['dom-overlay','tracked-sources'],domOverlay:{root:$('hud')}});
  session.addEventListener('inputsourceschange',describeSources);
  session.addEventListener('trackedsourceschange',describeSources);
  describeSources();
  connect();claim();
  const setup=await Promise.all([session.requestReferenceSpace('local'),gl.makeXRCompatible()]);space=setup[0];
  session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl,{framebufferScaleFactor:.65})});
  space.addEventListener('reset',()=>lost('跟踪坐标重定位'));
  session.addEventListener('visibilitychange',()=>{if(session?.visibilityState!=='visible')lost('头显暂时不可见');});
  session.addEventListener('end',()=>{lost('XR 结束');command('release');session=null;ready=false;claimSent=false;validBefore=false;started=0;document.body.classList.remove('xr-active');$('start').disabled=false;$('tracking').textContent='';});
  mark('xr_ready_ms');report('xr_ready');document.body.classList.add('xr-active');session.requestAnimationFrame(tick);$('status').textContent='请展示左手，自动开始跟随';
 }catch(e){report('xr_error',e.message);$('status').textContent=e.message;if(session){try{await session.end();}catch{}}session=null;$('start').disabled=false;}
 finally{entering=false;}
};
