var light;
var canvas = document.getElementById("Light");
var context = canvas.getContext("2d");
context.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);


if (typeof(Storage) !== "undefined") {
    if(localStorage.getItem("IP")!=null){
        document.getElementById("RobotIP").value = localStorage.getItem("IP");
    }else{
        document.getElementById("RobotIP").value = "localhost";
        localStorage.IP =  "localhost";
    }
    if(localStorage.getItem("Host")!=null){
        document.getElementById("RobotHost").value = localStorage.getItem("Host");
    }else{
        document.getElementById("RobotHost").value = "9090";
        localStorage.Host = "9090"
    }
}else{
  console.log('Sorry, your browser does not support Web Storage...');
}
//Robot_connnet
var ros = new ROSLIB.Ros({
  url : 'ws://'+document.getElementById("RobotIP").value+':'+document.getElementById("RobotHost").value
});
//confirm_connect
ros.on('connection', function() {
  console.log('Robot1 Connected to websocket server.');
  light = "connected";
  context.fillStyle = "blue";
  context.fill();
});
ros.on('error', function(error) {
  console.log('Robot1 Error connecting to websocket server:');
  light = "disconnected";
  context.fillStyle = "red";
  context.fill();
});
ros.on('close', function() {
  console.log('Robot1 Connection to websocket server closed.');
  light = "disconnected";
  context.fillStyle = "red";
  context.fill();
});

function RobotConnect()
{
  var IP = document.getElementById("RobotIP").value;
  if (IP != '') {
      localStorage.IP = IP;
  }else{
    if(localStorage.getItem("IP")!=null){
        IP = localStorage.getItem("IP");
    }else{
       IP = "localhost";
       localStorage.IP = "localhost";
    }
  }

  var Host = document.getElementById("RobotHost").value;
  if (Host != '') {
      localStorage.Host = Host;
  }else{
      if(localStorage.getItem("Host")!=null){
          Host = localStorage.getItem("Host");
      }else{
         Host = "9090";
         localStorage.Host = "9090";
      }
  }
  ros = new ROSLIB.Ros({
       url : 'ws://'+IP+':'+Host
  });
}



//confirm_connect
ros.on('connection', function() {
  console.log('Robot1 Connected to websocket server.');
  light = "connected";
  context.fillStyle = "blue";
  context.fill();
});
ros.on('error', function(error) {
  console.log('Robot1 Error connecting to websocket server:');
  light = "disconnected";
  context.fillStyle = "red";
  context.fill();
});
ros.on('close', function() {
  console.log('Robot1 Connection to websocket server closed.');
  light = "disconnected";
  context.fillStyle = "red";
  context.fill();
});

//==========================================================================
function circle(){
  var inner1 ={x:"-500",y:"130"};
  var inner2 ={x:"-500",y:"144"};
  var inner3 ={x:"500",y:"130"};
  var inner4 ={x:"500",y:"144"};

  var RobotCoordinate = {x:"125",y:"160"};

  var vector1 ={x:"0",y:"0"};
  var vector2 ={x:"0",y:"0"};
  var vector3 ={x:"0",y:"0"};
  var O1 ={x:"0",y:"0"};
  var O2 ={x:"0",y:"0"};

  var length1,length2,length3;
  var angle1,angle2,r1,r2;
  
  vector1.x = RobotCoordinate.x - inner1.x;
  vector1.y = RobotCoordinate.y - inner1.y;
  vector2.x = RobotCoordinate.x - inner3.x;
  vector2.y = RobotCoordinate.y - inner3.y;
  vector3.x = inner1.x - inner3.x;
  vector3.y = inner1.y - inner3.y;


  length1 = Math.pow(Math.pow(vector1.x,2)+Math.pow(vector1.y,2),0.5);
  length2 = Math.pow(Math.pow(vector2.x,2)+Math.pow(vector2.y,2),0.5);
  length3 = Math.pow(Math.pow(vector3.x,2)+Math.pow(vector3.y,2),0.5);
  
  if((vector2.x*vector3.y-vector2.y*vector3.x)<0){
    angle1 = Math.acos((vector2.x*vector3.x + vector2.y*vector3.y)/(length2*length3))*180/Math.PI;
  }else{
    angle1 = 360 - Math.acos((vector2.x*vector3.x + vector2.y*vector3.y)/(length2*length3))*180/Math.PI;
  }

  O1.x =(1/(Math.cos(DtoR(90-angle1))*2))*(Math.cos(DtoR(90-angle1))*vector1.x - Math.sin(DtoR(90-angle1))*vector1.y)+parseInt(inner1.x);
  O1.y =(1/(Math.cos(DtoR(90-angle1))*2))*(Math.sin(DtoR(90-angle1))*vector1.x + Math.cos(DtoR(90-angle1))*vector1.y)+parseInt(inner1.y);
  document.getElementById("O1").innerHTML="O1: (" +O1.x + "," + O1.y + ")";
  document.getElementById("O1").style.fontSize="xx-large";
  document.getElementById("O1").style.color = "#ffffff";

}

function DtoR(angle){
  return angle*Math.PI/180;
}




//==========================================================================\