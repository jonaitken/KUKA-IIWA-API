//
// kuka-iiwa-gui -- A GUI contoroller for KUKA-IIWA-API
// NISHI, Takao <zophos@ni.aist.go.jp>
//
'use strict';

//const DEBUG=true;
//const USE_MOCK_BRIDGE=true;

const Electron=require('electron');
const App=Electron.app;
const BrowserWindow=Electron.BrowserWindow;
const Ipc=Electron.ipcMain;


////////////////////////////////////////////////////////////////////////
//
// RosNode Bridge
//
const DEFAULT_TOOL='tool1';

function RosNodeBridge(recv)
{
    this.reciever=recv;
    this.rosnode=null;
    this.publisher=null;
    this.subscriber={};
    this.std_msgs=null;
}
if(typeof(USE_MOCK_BRIDGE)!='undefined' && USE_MOCK_BRIDGE){
    RosNodeBridge.prototype.start=function()
    {
	this.std_msgs={String:null};
	this.reciever.send('set-tool',DEFAULT_TOOL);
	this.reciever.send('pos-joint',
			   '[0.0, -1.0, 2.0, -3.0, 4.0, -5.0, 6.0] 7.0');
	this.reciever.send('pos-world',
			   '[0.0, 1.1, 2.2, 3.3, 4.4, 5.5] 6.6');
    }
    RosNodeBridge.prototype.post=function(cmd)
    {
	console.log(cmd);

	let m=cmd.match(/^setTool (\S+)/)
	if(m)
	    this.reciever.send('set-tool',m[1]);
    }
    RosNodeBridge.prototype.subscribe=function(topic,msg_type,func)
    {
	console.log('[sub]: '+topic);
    }
    RosNodeBridge.prototype.unsubscribe=function(topic)
    {
	console.log('[unsub]: '+topic);
    }
    
}
else{
    RosNodeBridge.prototype.start=function()
    {
	const RosNode=require('rosnodejs');

	RosNode.initNode(
	    '/kuka_iiwa_gui',
	    {onTheFly:true}).then(
		(rosNode)=>{
		    this.rosnode=rosNode;
		    this.std_msgs=RosNode.require('std_msgs').msg;

		    this.publisher=rosNode.advertise('/kuka_command',
						     this.std_msgs.String);
		    
		    this.subscribe('/hasError',
				   this.std_msgs.String,
				   (data)=>{
				       this.reciever.send('has-error',
							  data.data);
				   });
		    this.subscribe('/JointPosition',
				   this.std_msgs.String,
				   (data)=>{
				       this.reciever.send('pos-joint',
							  data.data);
				   });
		    this.subscribe('/ToolPosition',
				   this.std_msgs.String,
				   (data)=>{
				       this.reciever.send('pos-world',
							  data.data);
				   });

		    this.post('setTool '+DEFAULT_TOOL);
		}
	    );
    }
    RosNodeBridge.prototype.post=function(cmd)
    {
	const msg=new this.std_msgs.String();
	msg.data=cmd;
	this.publisher.publish(msg);
	
	let m=cmd.match(/^setTool (\S+)/)
	if(m)
	    this.reciever.send('set-tool',m[1]);
    }

    RosNodeBridge.prototype.subscribe=function(topic,
					       msg_type,
					       func,
					       option)
    {
	if(this.subscriber[topic])
	    return;

	this.subscriber[topic]=this.rosnode.subscribe(topic,
						      msg_type,
						      func,
						      option);
    }
    RosNodeBridge.prototype.unsubscribe=function(topic)
    {
	if(!this.subscriber[topic])
	    return;

	this.subscriber[topic].shutdown();

	delete this.subscriber[topic];
    }
}
RosNodeBridge.prototype.monitor_start_toolforce=function()
{
    this.subscribe('/ToolForce',
		   this.std_msgs.String,
		   (data)=>{
		       this.reciever.send('tool-force',data.data);
		   });
}
RosNodeBridge.prototype.monitor_stop_toolforce=function()
{
    this.unsubscribe('/ToolForce');
}
RosNodeBridge.prototype.monitor_start_tooltorque=function()
{
    this.subscribe('/ToolTorque',
		   this.std_msgs.String,
		   (data)=>{
		       this.reciever.send('tool-torque',data.data);
		   });
}
RosNodeBridge.prototype.monitor_stop_tooltorque=function()
{
    this.unsubscribe('/ToolTorque');
}


let mainWindow=null;

App.on('window-all-closed',()=>{
    if(process.platform!='darwin')
	App.quit();
});

App.on('ready',()=>{
    mainWindow=new BrowserWindow({width:960,
				  height:618,
				  useContentSize:true});
    if(typeof(DEBUG)!='undefined' && DEBUG)
        mainWindow.openDevTools();
    else
	mainWindow.setResizable(false);

    mainWindow.setMenu(null);
    mainWindow.loadURL('file://' + __dirname + '/index.html');
    mainWindow.on('closed',()=>{
	mainWindow=null;
    });
});

let rnb=null;
Ipc.on('READY',function(event,arg){
    if(!rnb){
	rnb=new RosNodeBridge(event.sender);
	rnb.start();
    }
});

Ipc.on('kuka-command',function(event,arg){
    if(rnb)
	rnb.post(arg);
});

Ipc.on('monitor-ext',function(event,arg){
    if(rnb){
	if(arg)
	    rnb.monitor_start_toolforce();
	else
	    rnb.monitor_stop_toolforce();
    }
});

Ipc.on('monitor-torque',function(event,arg){
    if(rnb){
	if(arg)
	    rnb.monitor_start_tooltorque();
	else
	    rnb.monitor_stop_tooltorque();
    }
});
