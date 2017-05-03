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

//Topic
//Region
var RegionBox = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/Location',
    messageType: 'std_msgs/Float32MultiArray'
});
//Optimization
var OptimizationBox = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/Optimization',
    messageType: 'std_msgs/Int32MultiArray'
});
//GameState
var GameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
//IsSimulator
var IsSimulator = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});
//TopicFunction
function TopicRegion(Num) {
    var box = new ROSLIB.Message({
        data: [Num[0], Num[1], Num[2], Num[3], Num[4], Num[5], Num[6], Num[7], Num[8], Num[9]]
    });
    RegionBox.publish(box);
}
function TopicOptimization(Num) {
    var box = new ROSLIB.Message({
        data: [Num[0], Num[1], Num[2], Num[3]]
    });
    OptimizationBox.publish(box);
}
function topicROSGameState(state) {
    console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    GameState.publish(gameState);
}
function OpenSimulator(checked) {
    var temp;
    if (checked == true) {
        temp = new ROSLIB.Message({
            data: 1
        });
        IsSimulator.publish(temp);
    } else {
        temp = new ROSLIB.Message({
            data: 0
        });
        IsSimulator.publish(temp);
    }
}
//========================================================
function RegionOrder(region, flag) {
    var i;
    var checked = 0;
    if (RegionTimes == -1) {
        Order[++RegionTimes] = region;
        Checkorder[RegionTimes] = flag;
    }
    for (i = 0; i < 5; i++) {
        if (Checkorder[i] == flag) {
            Order[i] = region;
            checked = 1;
        }
    }
    if (checked == 0) {
        Checkorder[++RegionTimes] = flag;
        Order[RegionTimes] = region;
    }
    console.log(Checkorder, Order);
}

function RegionReset() {
    RegionTimes = -1;
    for (i = 0; i < 5; i++) {
        Order[i] = 0;
        Checkorder[i] = 0;
    }
    console.log(Checkorder, Order);
}

function RegionLocation() {
    var i, j = -1;
    var Box = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (i = 0; i < 5; i++) {
        switch (Order[i]) {
            case 1:
                Box[++j] = parseFloat(-280);
                Box[++j] = parseFloat(180);
                break;
            case 2:
                Box[++j] = parseFloat(-75);
                Box[++j] = parseFloat(180);
                break;
            case 3:
                Box[++j] = parseFloat(75);
                Box[++j] = parseFloat(180);
                break;
            case 4:
                Box[++j] = parseFloat(280);
                Box[++j] = parseFloat(180);
                break;
            case 5:
                Box[++j] = parseFloat(-225);
                Box[++j] = parseFloat(105);
                break;
            case 6:
                Box[++j] = parseFloat(-122.5);
                Box[++j] = parseFloat(95);
                break;
            case 7:
                Box[++j] = parseFloat(0);
                Box[++j] = parseFloat(90);
                break;
            case 8:
                Box[++j] = parseFloat(122.5);
                Box[++j] = parseFloat(95);
                break;
            case 9:
                Box[++j] = parseFloat(225);
                Box[++j] = parseFloat(105);
                break;
            case 10:
                Box[++j] = parseFloat(-260);
                Box[++j] = parseFloat(0);
                break;
            case 11:
                Box[++j] = parseFloat(-165);
                Box[++j] = parseFloat(0);
                break;
            case 12:
                Box[++j] = parseFloat(165);
                Box[++j] = parseFloat(0);
                break;
            case 13:
                Box[++j] = parseFloat(260);
                Box[++j] = parseFloat(0);
                break;
            case 14:
                Box[++j] = parseFloat(-225);
                Box[++j] = parseFloat(-105);
                break;
            case 15:
                Box[++j] = parseFloat(-122.5);
                Box[++j] = parseFloat(-95);
                break;
            case 16:
                Box[++j] = parseFloat(0);
                Box[++j] = parseFloat(-90);
                break;
            case 17:
                Box[++j] = parseFloat(122.5);
                Box[++j] = parseFloat(-95);
                break;
            case 18:
                Box[++j] = parseFloat(225);
                Box[++j] = parseFloat(-105);
                break;
            case 19:
                Box[++j] = parseFloat(-280);
                Box[++j] = parseFloat(-180);
                break;
            case 20:
                Box[++j] = parseFloat(-75);
                Box[++j] = parseFloat(-180);
                break;
            case 21:
                Box[++j] = parseFloat(75);
                Box[++j] = parseFloat(-180);
                break;
            case 22:
                Box[++j] = parseFloat(280);
                Box[++j] = parseFloat(-180);
                break;
        }
    }
    for (i = 0; i < 10; i += 2) {
        var temp = 0;
        temp = Box[i];
        Box[i] = -Box[i + 1];
        Box[i + 1] = temp;
    }
    console.log(Box);
    DrawOrderRectangle();
    TopicRegion(Box);
}
//===========================================================
function Optimization() {
    var CheckBox = [];
    var obj = document.getElementsByName("OptimizationElement");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked) {
            CheckBox.push(parseInt(1));
        } else {
            CheckBox.push(parseInt(0));
        }
    }
    TopicOptimization(CheckBox);
}
//===========================================================
// parameter1
//General_variable
var SPlanningVelocityBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SPlanning_Velocity'
});
function GeneralTransfer(box1){
    SPlanningVelocityBox.set(box1);
    //DistanceSettingsBox.set(box2);
}
//GeneralGet
var obj;
SPlanningVelocityBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
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