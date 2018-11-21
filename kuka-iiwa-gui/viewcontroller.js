//
// viewcontroller.js -- a part of kuka-iiwa-gui
// NISHI, Takao <zophos@ni.aist.go.jp>
//
'use strict';

const Electron=require('electron');
const Ipc=Electron.ipcRenderer;
const Clipboard=Electron.clipboard;

//const Remote=Electron.remote;
//const ProductName=Remote.require('electron').app.getName();

const Util=require('./util');

////////////////////////////////////////////////////////////////////////
//
// history manager
//
function HistoryManager(max_sz)
{
    this.max_sz=max_sz;
    this.history=[];
    this.cursor=0;
}
HistoryManager.prototype.clear=function()
{
    this.history=[];
    this.cursor=0;
}
HistoryManager.prototype.rewind=function()
{
    this.cursor=this.history.length;
}

HistoryManager.prototype.add=function(item,rewind=false)
{
    if(this.max_sz && this.history.length>this.max_sz)
	this.history.shift();

    this.history.push(item);

    this.cursor=this.history.length;
    if(!rewind)
	this.cursor--;
}
HistoryManager.prototype.step_back=function()
{
    this.cursor--;
    if(this.cursor<0){
	this.cursor=0;
	return undefined;
    }

    return this.history[this.cursor];
}
HistoryManager.prototype.step_forward=function()
{
    this.cursor++;
    if(this.cursor>=this.history.length){
	this.cursor=this.history.length
	return undefined;
    }

    return this.history[this.cursor];
}
HistoryManager.prototype.step=function(forward)
{
    if(forward)
	return this.step_forward();
    else
	return this.step_back();
}
HistoryManager.prototype.is_backable=function()
{
    return this.cursor>0;
}
HistoryManager.prototype.is_forwardable=function()
{
    return this.cursor<this.history.length-1;
}

//
// end of HistoryManager
//
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
//
// Numeric format helper
//
Number.prototype.toFixedByName=function(x){
    if(x.match(/(^[xyz]$)|(\-[xyz]\-)/))
	return this.toFixed(1);
    else
	return this.toFixed(2);
}

////////////////////////////////////////////////////////////////////////
//
// view controller
//
const JOINT_NAMES=Object.freeze(['a1','a2','a3','a4','a5','a6','a7']);
const WORLD_NAMES=Object.freeze(['x','y','z','a','b','c']);
const TOOL_NAMES=Object.freeze(['x','y','z','a','b','c']);
const ORTHOGONAL_AXIS_NAMES=Object.freeze(['x','y','z']);
const TAB_NAMES=Object.freeze(['joint','world','tool','misc']);

const DEG_TICKS_NORMAL=1.0;
const DEG_TICKS_FINE=0.1;
const DEG_TICKS_ROUGH=5.0;

const MM_TICKS_NORMAL=5.0;
const MM_TICKS_FINE=1.0;
const MM_TICKS_ROUGH=25.0;

const HISTORY_SZ=1000;

const UPDATE_INTERVAL_MSEC=66.7; // 15fps
const MONITOR_TIMEOUT_MSEC=1000.0;
const MOUSEWHEEL_SENCE_INTERVAL_MSEC=200.0;

function ViewController()
{
    this.ticks={
	deg:{rough:DEG_TICKS_ROUGH,
	     normal:DEG_TICKS_NORMAL,
	     fine:DEG_TICKS_FINE},
	mm:{rough:MM_TICKS_ROUGH,
	    normal:MM_TICKS_NORMAL,
	    fine:MM_TICKS_FINE}
    };

    this.position={};
    JOINT_NAMES.forEach(function(v){
	this.position[v]=NaN;
    },this);
    WORLD_NAMES.forEach(function(v){
	this.position[v]=NaN;
    },this);

    this.force={
	ext:{},
	torque:{}
    };
    ORTHOGONAL_AXIS_NAMES.forEach(function(v){
	this.force.ext[v]=NaN;
	this.force.torque[v]=NaN;
    },this);

    this.history={
	joint:new HistoryManager(HISTORY_SZ),
	world:new HistoryManager(HISTORY_SZ),
	tool:new HistoryManager(HISTORY_SZ),
	misc:new HistoryManager(HISTORY_SZ*4)
    };

    this.last_update={
	joint:NaN,
	world:NaN,
	ext:NaN,
	torque:NaN
    };

    this._add_listeners();

    this.current_tool=null;
    this.current_tab=null;

    this.set_current_tab('joint');


    this._object_cache={};
    this.ext_monitor_off();
    this.torque_monitor_off();

    this.has_error=false;

    //
    // start monitor update thread
    //
    this._update_lock=null;
    this._update_timer=
	setInterval(()=>this._update(),UPDATE_INTERVAL_MSEC);
}

ViewController.prototype.set_current_tab=function(name,event)
{
    TAB_NAMES.forEach(function(tab_id){
	let thead=document.getElementById('region-command-tab-'+tab_id);
	let tbody=document.getElementById('region-command-body-'+tab_id);
	if(tab_id===name){
	    thead.setAttribute('class','enable');
	    let text=tbody.getAttribute('class');
	    text=text.replace(/enable/,'').trim();
	    tbody.setAttribute('class',text+' enable');
	}
	else{
	    thead.removeAttribute('class');
	    let text=tbody.getAttribute('class');
	    text=text.replace(/enable/,'').trim();
	    tbody.setAttribute('class',text);
	}
    },this);

    let button=document.getElementById('region-command-button-cval');
    if(name==='joint' || name==='world'){
	button.removeAttribute('class');
	button.setAttribute('tabindex','0');
    }
    else{
	button.setAttribute('class','disable');
	button.removeAttribute('tabindex');
    }

    this.history[name].rewind();
    this.current_tab=name;
    this._set_history_button_status();
}

ViewController.prototype.ext_monitor_on=function()
{
    this._set_force_monitor_status('ext',true);
}
ViewController.prototype.ext_monitor_off=function()
{
    this._set_force_monitor_status('ext',false);
}
ViewController.prototype.ext_monitor_toggle=function()
{
    this._toggle_force_monitor_status('ext');
}

ViewController.prototype.torque_monitor_on=function()
{
    this._set_force_monitor_status('torque',true);
}
ViewController.prototype.torque_monitor_off=function()
{
    this._set_force_monitor_status('torque',false);
}
ViewController.prototype.torque_monitor_toggle=function()
{
    this._toggle_force_monitor_status('torque');
}

ViewController.prototype.set_current_tool=function(name,event)
{
    this.current_tool=name;

    let obj=document.getElementById('tool-name');
    obj.innerText=Util.escapeHTML(name);
}

ViewController.prototype.joint_step=function(name,sign,event)
{
    let val=this._get_step(name,event);
    val*=sign;

    let tmp=JOINT_NAMES.map(function(v,idx,src){
	if(name==v)
	    return (val+this.position[name]);
	else
	    return '-';
    },this);

    this.post_command('setPosition '+tmp.join(' '));
}


ViewController.prototype.world_step=function(name,sign,event)
{
    let val=this._get_step(name,event);
    val*=sign;

    let tmp=WORLD_NAMES.map(function(v,idx,src){
	if(name==v)
	    return (val+this.position[name]);
	else
	    return '-';
    },this);

    tmp.push('lin');

    this.post_command('setPositionXYZABC '+tmp.join(' '));
}

ViewController.prototype.post_forceStop=function()
{
    this.post_command('forceStop');
}

ViewController.prototype.post_command=function(cmd)
{
    Ipc.send('kuka-command',cmd);
}

ViewController.prototype.update_position=function(str,names)
{
    let m=str.match(/\[([^\]]+)\]/);
    if(!m)
	return;

    let now=Date.now();

    let val=m[1].split(',').map(x=>parseFloat(x));
    names.forEach(function(v,i){
	this.position[v]=val[i];
    },this);

    if(names[0]==='a1')
	this.last_update['joint']=now;
    else
	this.last_update['world']=now;
}

ViewController.prototype.update_force=function(str,name)
{
    let m=str.match(/\[([^\]]+)\]/);
    if(!m)
	return;

    let now=Date.now();

    let val=m[1].split(',').map(x=>parseFloat(x));
    ORTHOGONAL_AXIS_NAMES.forEach(function(v,i){
	this.force[name][v]=val[i];
    },this);

    this.last_update[name]=now;
}

ViewController.prototype.apply_history=function(forward)
{
    let item=this.history[this.current_tab].step(forward);
    this._set_history_button_status();
    if(!item){
	if(forward)
	    item=[];
	else
	    return;
    }

    switch(this.current_tab){
    case 'joint':
	JOINT_NAMES.forEach(function(v,i){
	    let name='joint-'+v+'-cmd';
	    document.getElementById(name).innerText=item[i]||'-';
	},this);
	break;
    case 'world':
	WORLD_NAMES.forEach(function(v,i){
	    let name='world-'+v+'-cmd';
	    document.getElementById(name).innerText=item[i]||'-';
	},this);
	let el=document.getElementsByName('world-m-cmd');
	if(item[item.length-1]===el[1].value){
	    el[0].checked=false;
	    el[1].checked=true;
	}
	else{
	    el[0].checked=true;
	    el[1].checked=false;
	}
	break;
    case 'tool':
	TOOL_NAMES.forEach(function(v,i){
	    let name='tool-'+v+'-cmd';
	    document.getElementById(name).innerText=item[i]||'0';
	},this);
	break;
    case 'misc':
	let o=document.getElementById('direct-cmd-text');
	if(Array.isArray(item))
	    o.value=item[0]||null;
	else
	    o.value=item;
	break;
    }
}



ViewController.prototype._add_listeners=function()
{
    //
    // set joint inc/dec event
    //
    JOINT_NAMES.forEach(function(v){
	let name='joint-'+v+'-dec';
	document.getElementById(name).addEventListener(
	    'click',
	    this.joint_step.bind(this,v,-1.0)
	);
	name='joint-'+v+'-inc';
	document.getElementById(name).addEventListener(
	    'click',
	    this.joint_step.bind(this,v,1.0)
	);
    },this);

    //
    // set world inc/dec event
    //
    WORLD_NAMES.forEach(function(v){
	let name='world-'+v+'-dec';
	document.getElementById(name).addEventListener(
	    'click',
	    this.world_step.bind(this,v,-1.0)
	);
	name='world-'+v+'-inc';
	document.getElementById(name).addEventListener(
	    'click',
	    this.world_step.bind(this,v,1.0)
	);
    },this);

    //
    // set force monitors event
    //
    document.getElementById('ext-caption').addEventListener(
	'click',
	this.ext_monitor_toggle.bind(this)
    );
    document.getElementById('torque-caption').addEventListener(
	'click',
	this.torque_monitor_toggle.bind(this)
    );

    //
    // set tab event
    //
    TAB_NAMES.forEach(function(tab_id){
	let name='region-command-tab-'+tab_id;
	document.getElementById(name).addEventListener(
	    'click',
	    this.set_current_tab.bind(this,tab_id)
	);
    },this);
    
    //
    // set joint position input event
    //
    JOINT_NAMES.forEach(function(v){
	let name='joint-'+v+'-cmd';
	let obj=document.getElementById(name);
	obj.addEventListener(
	    'keypress',
	    this._check_digit_key
	);
	obj.addEventListener(
	    'blur',
	    this._check_digit_value.bind(obj,'-')
	);
    },this);

    //
    // set world position input event
    //
    WORLD_NAMES.forEach(function(v){
	let name='world-'+v+'-cmd';
	let obj=document.getElementById(name);
	obj.addEventListener(
	    'keypress',
	    this._check_digit_key
	);
	obj.addEventListener(
	    'blur',
	    this._check_digit_value.bind(obj,'-')
	);
    },this);

    //
    // set tool position input event
    //
    TOOL_NAMES.forEach(function(v){
	let name='tool-'+v+'-cmd';
	let obj=document.getElementById(name);
	obj.addEventListener(
	    'keypress',
	    this._check_digit_key
	);
	obj.addEventListener(
	    'blur',
	    this._check_digit_value.bind(obj,'0')
	);
    },this);
    
    //
    // zero clear value button event
    //
    let obj=document.getElementById('region-command-button-setzero');
    obj.addEventListener(
	'keypress',
	(event)=>{
	    if(event.key=='Enter')
		this._zero_clear_value.call(this,event);
	}
    );
    obj.addEventListener(
	'click',
	this._zero_clear_value.bind(this)
    );

    //
    // import current value button event
    //
    obj=document.getElementById('region-command-button-cval');
    obj.addEventListener(
	'keypress',
	(event)=>{
	    if(event.key=='Enter')
		this._import_current_value.call(this,event);
	}
    );
    obj.addEventListener(
	'click',
	this._import_current_value.bind(this)
    );

    //
    // step back/forward history button event
    //
    obj=document.getElementById('region-command-button-hist-older');
    obj.addEventListener(
	'keypress',
	(event)=>{
	    if(event.key=='Enter')
		this.apply_history.bind(this,null);
	}
    );
    obj.addEventListener(
	'click',
	this.apply_history.bind(this,null)
    );
    obj=document.getElementById('region-command-button-hist-newer');
    obj.addEventListener(
	'keypress',
	(event)=>{
	    if(event.key=='Enter')
		this.apply_history.bind(this,true);
	}
    );
    obj.addEventListener(
	'click',
	this.apply_history.bind(this,true)
    );
    
    //
    // direct command box event
    //
    document.getElementById('direct-cmd-text').addEventListener(
	'keypress',
	(event)=>{
	    switch(event.key){
	    case 'Enter':
		this._exec_cmd.call(this,event);
		break;
	    case 'ArrowUp':
		this.apply_history();
		break;
	    case 'ArrowDown':
		//
		// this will conflict with autocompletion.
		//
		this.apply_history(true);
		break;
	    }
	}
    );

    //
    // exec button event
    //
    obj=document.getElementById('region-command-button-exec');
    obj.addEventListener(
	'keypress',
	(event)=>{
	    if(event.key=='Enter')
		this._exec_cmd.call(this,event);
	}
    );
    obj.addEventListener(
	'click',
	this._exec_cmd.bind(this)
    );

    //
    // don't panic button event
    //
    obj=document.getElementById('do-not-panic');
    obj.addEventListener(
	'keypress',
	this.post_forceStop.bind(this)
    );
    obj.addEventListener(
	'click',
	this.post_forceStop.bind(this)
    );

    //
    // try recover button event
    //
    document.getElementById('motion-error-recover').addEventListener(
	'click',
	this._try_recover.bind(this)
    );


    //
    // hooks to global window event
    //
    window.addEventListener(
	'keydown',
	(event)=>{
	    switch(event.key){
	    case 'Escape':
		this.post_forceStop();
		break;
	    case 'ArrowUp':
		this.apply_history();
		break;
	    case 'ArrowDown':
		this.apply_history(true);
		break;
	    case 'c':
		if(event.ctrlKey)
		    this._copy_current_values();
		break;
	    default:
		//console.log(event);
	    }
	}
    );

    this._mousewheel_timer=null;
    window.addEventListener(
	'mousewheel',
	(event)=>{
	    if(this._mousewheel_timer)
		return;

	    if(event.wheelDelta>0)
		this.apply_history();
	    else if(event.wheelDelta<0)
		this.apply_history(true);

	    this._mousewheel_timer=setTimeout(
		()=>this._mousewheel_timer=null,
		MOUSEWHEEL_SENCE_INTERVAL_MSEC);
	}
    );
}

ViewController.prototype._get_step=function(name,event)
{
    let unit='deg';
    if(name==='x'||name==='y'||name==='z')
	unit='mm';

    if(event.shiftKey)
	return this.ticks[unit].rough;
    else if(event.ctrlKey)
	return this.ticks[unit].fine;
    else
	return this.ticks[unit].normal;
}

ViewController.prototype._check_digit_key=function(event)
{
    event.stopPropagation();

    //
    // allow less than 8 chars digit, period or sign.
    //
    if(event.key){
	let obj=event.currentTarget;
	if(event.key=='Enter')
	    obj.blur();
	else if(event.key=='='){
	    let src=document.getElementById(obj.id.replace('-cmd','-val'));
	    if(src)
		obj.innerText=src.innerText;
	}
	else if(obj.innerText.length<8){
	    if(event.key.match(/[\d\.\-]/))
		return;
	}
    }
    event.preventDefault();
}

ViewController.prototype._check_digit_value=function(fallback_char,event)
{
    let num=parseFloat(this.innerText);
    if(isNaN(num))
	this.innerText=fallback_char;
    else
	this.innerText=num.toFixedByName(this.id);
}

ViewController.prototype._zero_clear_value=function(event)
{
    switch(this.current_tab){
    case 'joint':
	JOINT_NAMES.forEach(function(v){
	    let dst=document.getElementById('joint-'+v+'-cmd');
	    dst.innerText='-';
	},this);
	break;
    case 'world':
	WORLD_NAMES.forEach(function(v){
	    let dst=document.getElementById('world-'+v+'-cmd');
	    dst.innerText='-';
	},this);
	break;
    case 'tool':
	TOOL_NAMES.forEach(function(v,i){
	    let name='tool-'+v+'-cmd';
	    document.getElementById(name).innerText='0';
	},this);
	break;
    case 'misc':
	let o=document.getElementById('direct-cmd-text');
	o.value=null;
	break;
    }
}

ViewController.prototype._import_current_value=function(event)
{
    switch(this.current_tab){
    case 'joint':
	JOINT_NAMES.forEach(function(v){
	    let dst=document.getElementById('joint-'+v+'-cmd');
	    dst.innerText=this.position[v].toFixedByName(v);
	},this);
	break;
    case 'world':
	WORLD_NAMES.forEach(function(v){
	    let dst=document.getElementById('world-'+v+'-cmd');
	    dst.innerText=this.position[v].toFixedByName(v);
	},this);
	break;
    }
}

ViewController.prototype._set_force_monitor_status=function(name,status)
{
    this._set_force_monitor_display(name,status);
    this.force[name]['enable']=status||false;
    
    Ipc.send('monitor-'+name,this.force[name]['enable']);
}
ViewController.prototype._toggle_force_monitor_status=function(name)
{
    this._set_force_monitor_status(
	name,
	this.force[name]['enable'] ? false : true);
}
ViewController.prototype._set_force_monitor_display=function(name,status)
{
    if(!this._object_cache[name])
	this._object_cache[name]=document.getElementsByClassName(name);

    for(let i=0;i<this._object_cache[name].length;i++){
	let el=this._object_cache[name][i];
	let text=el.getAttribute('class');
	text=text.replace(/disable/,'').trim();
	if(!status)
	    text+=' disable';
	el.setAttribute('class',text);
    }
}

ViewController.prototype._set_position_monitor_display=function(name,status)
{
    let el=document.getElementById('tbl-'+name);
    if(status)
	el.removeAttribute('class');
    else
	el.setAttribute('class','disable');
}

ViewController.prototype._exec_cmd=function(event)
{
    let tmp=null;
    switch(this.current_tab){
    case 'joint':
	tmp=JOINT_NAMES.map(function(v){
	    let dst=document.getElementById('joint-'+v+'-cmd');
	    this._check_digit_value.call(dst,'-');
	    return dst.innerText;
	},this);
	this.history[this.current_tab].add(tmp.concat());
	tmp.unshift('setPosition');
	break;
    case 'world':
	tmp=WORLD_NAMES.map(function(v){
	    let dst=document.getElementById('world-'+v+'-cmd');
	    this._check_digit_value.call(dst,'-');
	    return dst.innerText;
	},this);
	let el=document.getElementsByName('world-m-cmd');
	if(el[0].checked)
	    tmp.push(el[0].value);
	else
	    tmp.push(el[1].value);
	this.history[this.current_tab].add(tmp.concat());
	tmp.unshift('setPositionXYZABC');
	break;
    case 'tool':
	tmp=TOOL_NAMES.map(function(v){
	    let dst=document.getElementById('tool-'+v+'-cmd');
	    this._check_digit_value.call(dst,'0');
	    return dst.innerText;
	},this);
	this.history[this.current_tab].add(tmp.concat());
	tmp.unshift('MoveXYZABC');
	break;
    case 'misc':
	let o=document.getElementById('direct-cmd-text');
	let ov=o.value;
	if(ov){
	    if(ov.indexOf("gui")==0){
		this.history[this.current_tab].add(ov,true);
		this._gui_setting(ov);
	    }
	    else
		tmp=[ov];
	}
	o.value=null;
	break;
    }

    this._set_history_button_status();
    if(tmp){
	let cmd=tmp.join(' ');
	this.history['misc'].add(cmd,true);
	this.post_command(cmd);
    }
}

ViewController.prototype._gui_setting=function(cmd)
{
    let argv=cmd.toLowerCase().split(' ');
    switch(argv[0]){
    case 'guisetticksmm':
	this._gui_setting_ticks(this.ticks.mm,argv.slice(1));
	break;
    case 'guisetticksdeg':
	this._gui_setting_ticks(this.ticks.deg,argv.slice(1));
	break;
    case 'guimonitortoolforce':
	switch(argv[1]){
	case 'on':
	case 'start':
	    this.ext_monitor_on();
	    break;
	case 'off':
	case 'stop':
	    this.ext_monitor_off();
	    break;
	}
	break;
    case 'guimonitortooltorque':
	switch(argv[1]){
	case 'on':
	case 'start':
	    this.torque_monitor_on();
	    break;
	case 'off':
	case 'stop':
	    this.torque_monitor_off();
	    break;
	}
	break;
    }
}
ViewController.prototype._gui_setting_ticks=function(obj,argv)
{
    if(val.length<3)
	return;

    let val=argv.map(x=>parseFloat(x));
    if(isFinite(val[0]))
       obj.fine=val[0];
    if(isFinite(val[1]))
       obj.normal=val[1];
    if(isFinite(val[2]))
       obj.rough=val[2];
}
ViewController.prototype._try_recover=function(event)
{
    let tmp=JOINT_NAMES.map(v=>this.position[v]);
    tmp.unshift('setPosition');
    this.post_command(tmp.join(' '));
}

ViewController.prototype._update=function()
{
    if(this._update_lock)
	return;
    this._update_lock=true;

    let now=Date.now();

    JOINT_NAMES.forEach(function(v){
	let obj=document.getElementById('joint-'+v+'-val');
	obj.innerText=this.position[v].toFixedByName(v);
    },this);
    WORLD_NAMES.forEach(function(v){
	let obj=document.getElementById('world-'+v+'-val');
	obj.innerText=this.position[v].toFixedByName(v);
    },this);
    ORTHOGONAL_AXIS_NAMES.forEach(function(v){
	document.getElementById('ext-'+v+'-val').innerText=
	    this.force.ext[v].toFixed(3);
	document.getElementById('torque-'+v+'-val').innerText=
	    this.force.torque[v].toFixed(3);
    },this);

    ['joint','world'].forEach(function(n){
	this._set_position_monitor_display(
	    n,
	    (now-this.last_update[n]<=MONITOR_TIMEOUT_MSEC));
    },this);
    ['ext','torque'].forEach(function(n){
	this._set_force_monitor_display(
	    n,
	    this.force[n]['enable'] && 
		(now-this.last_update[n]<=MONITOR_TIMEOUT_MSEC));
    },this);

    if(this.has_error){
	document.getElementById('do-not-panic').setAttribute('class','error');
	document.getElementById('motion-error').setAttribute('class','error');
    }
    else{
	document.getElementById('do-not-panic').removeAttribute('class');
	document.getElementById('motion-error').removeAttribute('class');
    }

    this._update_lock=null;
}

ViewController.prototype.set_autocomplete_list=function(items)
{
    let src=document.getElementById('cmd-complete-list');
    items.forEach(function(opt){
	let el=document.createElement('option');
	el.setAttribute('value',opt);
	src.appendChild(el);
    },this);
}

ViewController.prototype._set_history_button_status=function()
{
    let el=document.getElementById('region-command-button-hist-older');
    if(this.history[this.current_tab].is_backable()){
	el.removeAttribute('class');
	el.setAttribute('tabIndex','0');
    }
    else{
	el.setAttribute('class','disable');
	el.removeAttribute('tabIndex');
    }

    el=document.getElementById('region-command-button-hist-newer');
    if(this.history[this.current_tab].is_forwardable()){
	el.removeAttribute('class');
	el.setAttribute('tabIndex','0');
    }
    else{
	el.setAttribute('class','disable');
	el.removeAttribute('tabIndex');
    }
}
ViewController.prototype.set_error_status=function(str)
{
    if(str.match(/True/))
	this.has_error=true;
    else
	this.has_error=false;
}
ViewController.prototype._copy_current_values=function()
{
    Clipboard.writeText(JSON.stringify({position: this.position,
				       force: this.force})+"\n");
}

//
// end of ViewController
//
////////////////////////////////////////////////////////////////////////

window.onload=function(){
    document.controller=new ViewController();

    try{
	document.controller.set_autocomplete_list(
	    JSON.parse(require('fs').readFileSync('autocomplete.json','UTF-8'))
	);
    }
    catch(e){
	// ignore
    }

    Ipc.on('pos-joint',(event,arg)=>{
	document.controller.update_position(arg,JOINT_NAMES);
    });
    Ipc.on('pos-world',(event,arg)=>{
	document.controller.update_position(arg,WORLD_NAMES);
    });
    Ipc.on('tool-force',(event,arg)=>{
	document.controller.update_force(arg,'ext');
    });
    Ipc.on('tool-torque',(event,arg)=>{
	document.controller.update_force(arg,'torque');
    });
    Ipc.on('set-tool',(event,arg)=>{
	document.controller.set_current_tool(arg);
    });
    Ipc.on('has-error',(event,arg)=>{
	document.controller.set_error_status(arg);
    });
    Ipc.send('READY');
}
