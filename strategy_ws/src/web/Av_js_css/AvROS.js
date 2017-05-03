var light;
var canvas = document.getElementById("Light");
var canvas2 = document.getElementById("Light2");
var context = canvas.getContext("2d");
var context2 = canvas2.getContext("2d");
context.font = '30pt Calibri';
context2.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);
context2.arc(20, 20, 20, 0, Math.PI * 2, false);


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
//===========================================================
//Topic
//GameState
var TopicGameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
//IsSimulator
var TopicIsSimulator = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});

function topicROSGameState(state) {
    var gameState = new ROSLIB.Message({
        data: state
    });
    TopicGameState.publish(gameState);
}

function OpenSimulator(checked) {
    var temp;
    if (checked == true) {
        temp = new ROSLIB.Message({
            data: 1
        });
        TopicIsSimulator.publish(temp);
    } else {
        temp = new ROSLIB.Message({
            data: 0
        });
        TopicIsSimulator.publish(temp);
    }
}
//===
//===========================================================
// parameter1
//General_variable
var ParamSPlanningVelocityBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/SPlanning_Velocity'
});
var ParamDistantBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/Distant'
});
var ParamScanLineBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/Line'
});
var ParamChooseSideBox = new ROSLIB.Param({
    ros: ros,
    name: '/AvoidChallenge/ChooseSide'
});

function GeneralTransfer(box1, box2, box3, box4) {
    ParamSPlanningVelocityBox.set(box1);
    ParamDistantBox.set(box2);
    ParamScanLineBox.set(box3);
    ParamChooseSideBox.set(box4);
}
//GeneralGet
var obj;
ParamSPlanningVelocityBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ParamDistantBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DistantElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ParamScanLineBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ScanLineElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ParamChooseSideBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ChooseSideElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});


//service
// -----------------

var updateClient = new ROSLIB.Service({
    ros: ros,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});

var request = new ROSLIB.ServiceRequest({
    receive: 1
});

function up() {
    document.getElementById("Update").style.cursor = "wait";
    context2.fillStyle = "yellow";
    context2.fill();
    updateClient.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context2.fillStyle = "green";
            context2.fill();
        }
    });

}
