var light;
var canvas = document.getElementById("Light");
var context = canvas.getContext("2d");
context.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);


if (typeof(Storage) !== "undefined") {
    if (localStorage.getItem("IP") != null) {
        document.getElementById("RobotIP").value = localStorage.getItem("IP");
    } else {
        document.getElementById("RobotIP").value = "localhost";
        localStorage.IP = "localhost";
    }
    if (localStorage.getItem("Host") != null) {
        document.getElementById("RobotHost").value = localStorage.getItem("Host");
    } else {
        document.getElementById("RobotHost").value = "9090";
        localStorage.Host = "9090"
    }
} else {
    console.log('Sorry, your browser does not support Web Storage...');
}
//Robot_connnet
var ros = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP").value + ':' + document.getElementById("RobotHost").value
});
//confirm_connect
ros.on('connection', function() {
    console.log('Robot1 Connected to websocket server.');
    light = "connected";
    context.fillStyle = "green";
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

function RobotConnect() {
    var IP = document.getElementById("RobotIP").value;
    if (IP != '') {
        localStorage.IP = IP;
    } else {
        if (localStorage.getItem("IP") != null) {
            IP = localStorage.getItem("IP");
        } else {
            IP = "localhost";
            localStorage.IP = "localhost";
        }
    }

    var Host = document.getElementById("RobotHost").value;
    if (Host != '') {
        localStorage.Host = Host;
    } else {
        if (localStorage.getItem("Host") != null) {
            Host = localStorage.getItem("Host");
        } else {
            Host = "9090";
            localStorage.Host = "9090";
        }
    }
    ros = new ROSLIB.Ros({
        url: 'ws://' + IP + ':' + Host
    });
}

//confirm_connect
ros.on('connection', function() {
    console.log('Robot1 Connected to websocket server.');
    light = "connected";
    context.fillStyle = "green";
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

//Topic
//Level
var TopicLevel = new ROSLIB.Topic({
    ros: ros,
    name: '/passingChallenge/level',
    messageType: 'std_msgs/Int32'
});
//Postion
var TopicPosition = new ROSLIB.Topic({
    ros: ros,
    name: '/passingChallenge/robotLocation',
    messageType: 'geometry_msgs/Twist'
});
//cmd_vel
var TopicCmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});
//Shoot
var TopicShoot = new ROSLIB.Topic({
    ros: ros,
    name: '/shoot',
    messageType: 'std_msgs/Int32'
});
//Speed
var TopicSpeed = new ROSLIB.Topic({
    ros: ros,
    name: '/passingChallenge/Speed',
    messageType: 'std_msgs/Int32'
});

function topicROSLevel(state) {
    console.log(state);
    var level = new ROSLIB.Message({
        data: state
    });
    TopicLevel.publish(level);
}

TopicPosition.subscribe(function(message) {
    SavePosition(message.linear.x, message.linear.y, message.angular.z);
});
TopicCmdVel.subscribe(function(message) {
    SaveCmdVel(message.linear.x, message.linear.y, message.angular.z);
});
TopicShoot.subscribe(function(message) {
    SaveShoot(message.data);
});

function SavePosition(X, Y, Z) {
    if ((Robpos[0] != X) || (Robpos[1] != Y) || (Robpos[2] != Z)) {
        Robpos[0] = X;
        Robpos[1] = Y;
        Robpos[2] = Z;
        world_Location();
    }
}

function SaveCmdVel(X, Y, Z) {
    if ((Robcmdvel[0] != X) || (Robcmdvel[1] != Y) || (Robcmdvel[2] != Z)) {
        Robcmdvel[0] = X;
        Robcmdvel[1] = Y;
        Robcmdvel[2] = Z;
        world_Location();
    }
}

function SaveShoot(value) {
    if (Robshoot != value) {
        Robshoot = value;
        world_Location();
    }
}

function topicROSSpeed(Num) {
    var speed = new ROSLIB.Message({
        data: Num
    });
    TopicSpeed.publish(speed);
}
//===================================================
//Parameter
var X_Speed = new ROSLIB.Param({
    ros: ros,
    name: '/passingChallenge/X_speedConst'
});
var Y_Speed = new ROSLIB.Param({
    ros: ros,
    name: '/passingChallenge/Y_speedConst'
});
var Z_Speed = new ROSLIB.Param({
    ros: ros,
    name: '/passingChallenge/Yaw_speedConst'
});
var end_pos = new ROSLIB.Param({
    ros: ros,
    name: '/passingChallenge/endPos'
});
var ParamShoot = new ROSLIB.Param({
    ros: ros,
    name: '/passingChallenge/shootConst'
});

function GeneralTransfer(box1) {
    X_Speed.set(box1[0]);
    Y_Speed.set(box1[1]);
    Z_Speed.set(box1[2]);
    end_pos.set(box1[3]);
    ParamShoot.set(box1[4]);
}

var obj;
X_Speed.get(function(value) {
    obj = document.getElementsByName("ParameterElement");
    obj[0].value = value;
});
Y_Speed.get(function(value) {
    obj = document.getElementsByName("ParameterElement");
    obj[1].value = value;
});
Z_Speed.get(function(value) {
    obj = document.getElementsByName("ParameterElement");
    obj[2].value = value;
});
end_pos.get(function(value) {
    obj = document.getElementsByName("ParameterElement");
    obj[3].value = value;
});
ParamShoot.get(function(value) {
    obj = document.getElementsByName("ParameterElement");
    obj[4].value = value;
});
