//ROS_connect
var light, light2, light3;
var canvas = document.getElementById("Light");
var canvas2 = document.getElementById("Light2");
var canvas3 = document.getElementById("Light3");
var canvas4 = document.getElementById("Light4");
var context = canvas.getContext("2d");
var context2 = canvas2.getContext("2d");
var context3 = canvas3.getContext("2d");
var context4 = canvas4.getContext("2d");
context.font = '30pt Calibri';
context2.font = '30pt Calibri';
context3.font = '30pt Calibri';
context4.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);
context2.arc(20, 20, 20, 0, Math.PI * 2, false);
context3.arc(20, 20, 20, 0, Math.PI * 2, false);
context4.arc(20, 20, 20, 0, Math.PI * 2, false);

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

    if (localStorage.getItem("IP2") != null) {
        document.getElementById("RobotIP2").value = localStorage.getItem("IP2");
    } else {
        document.getElementById("RobotIP2").value = "localhost";
        localStorage.IP2 = "localhost";
    }
    if (localStorage.getItem("Host2") != null) {
        document.getElementById("RobotHost2").value = localStorage.getItem("Host2");
    } else {
        document.getElementById("RobotHost2").value = "9090";
        localStorage.Host2 = "9090"
    }

    if (localStorage.getItem("IP3") != null) {
        document.getElementById("RobotIP3").value = localStorage.getItem("IP3");
    } else {
        document.getElementById("RobotIP3").value = "localhost";
        localStorage.IP3 = "localhost";
    }
    if (localStorage.getItem("Host3") != null) {
        document.getElementById("RobotHost3").value = localStorage.getItem("Host3");
    } else {
        document.getElementById("RobotHost3").value = "9090";
        localStorage.Host3 = "9090"
    }
} else {
    console.log('Sorry, your browser does not support Web Storage...');
}


//Robot_connnet
var ros = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP").value + ':' + document.getElementById("RobotHost").value
});
var ros2 = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP2").value + ':' + document.getElementById("RobotHost2").value
});
var ros3 = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP3").value + ':' + document.getElementById("RobotHost3").value
});

//confirm_connect
ros.on('connection', function() {
    console.log('Robot1 Connected to websocket server.');
    light = "connected";
    context.fillStyle = "green";
    context.fill();
    CheckIP[0] = 1;
});
ros.on('error', function(error) {
    console.log('Robot1 Error connecting to websocket server:');
    light = "disconnected";
    context.fillStyle = "red";
    context.fill();
    CheckIP[0] = 0;
});
ros.on('close', function() {
    console.log('Robot1 Connection to websocket server closed.');
    light = "disconnected";
    context.fillStyle = "red";
    context.fill();
    CheckIP[0] = 0;
});

ros2.on('connection', function() {
    console.log('Robot2 Connected to websocket server.');
    light2 = "connected";
    context2.fillStyle = "green";
    context2.fill();
    CheckIP[1] = 1;
});
ros2.on('error', function(error) {
    console.log('Robot2 Error connecting to websocket server:');
    light2 = "disconnected";
    context2.fillStyle = "red";
    context2.fill();
    CheckIP[1] = 0;
});
ros2.on('close', function() {
    console.log('Robot2 Connection to websocket server closed.');
    light2 = "disconnected";
    context2.fillStyle = "red";
    context2.fill();
    CheckIP[1] = 0;
});

ros3.on('connection', function() {
    console.log('Robot3 Connected to websocket server.');
    light3 = "connected";
    context3.fillStyle = "green";
    context3.fill();
    CheckIP[2] = 1;
});
ros3.on('error', function(error) {
    console.log('Robot3 Error connecting to websocket server:');
    light3 = "disconnected";
    context3.fillStyle = "red";
    context3.fill();
    CheckIP[2] = 0;
});
ros3.on('close', function() {
    console.log('Robot3 Connection to websocket server closed.');
    light3 = "disconnected";
    context3.fillStyle = "red";
    context3.fill();
    CheckIP[2] = 0;
});

//When Connect bottom is pressed
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
    var IP2 = document.getElementById("RobotIP2").value;
    if (IP2 != '') {
        localStorage.IP2 = IP2;
    } else {
        if (localStorage.getItem("IP2") != null) {
            IP2 = localStorage.getItem("IP2");
        } else {
            IP2 = "localhost";
            localStorage.IP2 = "localhost";
        }
    }
    var IP3 = document.getElementById("RobotIP3").value;
    if (IP3 != '') {
        localStorage.IP3 = IP3;
    } else {
        if (localStorage.getItem("IP3") != null) {
            IP3 = localStorage.getItem("IP3");
        } else {
            IP3 = "localhost";
            localStorage.IP3 = "localhost";
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
    var Host2 = document.getElementById("RobotHost2").value;
    if (Host2 != '') {
        localStorage.Host2 = Host2;
    } else {
        if (localStorage.getItem("Host2") != null) {
            Host2 = localStorage.getItem("Host2");
        } else {
            Host2 = "9090";
            localStorage.Host2 = "9090";
        }
    }
    var Host3 = document.getElementById("RobotHost3").value;
    if (Host3 != '') {
        localStorage.Host3 = Host3;
    } else {
        if (localStorage.getItem("Host3") != null) {
            Host3 = localStorage.getItem("Host3");
        } else {
            Host = "9090";
            localStorage.Host3 = "9090";
        }
    }

    ros = new ROSLIB.Ros({
        url: 'ws://' + IP + ':' + Host
    });
    ros2 = new ROSLIB.Ros({
        url: 'ws://' + IP2 + ':' + Host2
    });
    ros3 = new ROSLIB.Ros({
        url: 'ws://' + IP3 + ':' + Host3
    });

    //confirm_connect
    ros.on('connection', function() {
        console.log('Robot1 Connected to websocket server.');
        light = "connected";
        context.fillStyle = "green";
        context.fill();
        CheckIP[0] = 1;
    });
    ros.on('error', function(error) {
        console.log('Robot1 Error connecting to websocket server:');
        light = "disconnected";
        context.fillStyle = "red";
        context.fill();
        CheckIP[0] = 0;
    });
    ros.on('close', function() {
        console.log('Robot1 Connection to websocket server closed.');
        light = "disconnected";
        context.fillStyle = "red";
        context.fill();
        CheckIP[0] = 0;
    });

    ros2.on('connection', function() {
        console.log('Robot2 Connected to websocket server.');
        light2 = "connected";
        context2.fillStyle = "green";
        context2.fill();
        CheckIP[1] = 1;
    });
    ros2.on('error', function(error) {
        console.log('Robot2 Error connecting to websocket server:');
        light2 = "disconnected";
        context2.fillStyle = "red";
        context2.fill();
        CheckIP[1] = 0;
    });
    ros2.on('close', function() {
        console.log('Robot2 Connection to websocket server closed.');
        light2 = "disconnected";
        context2.fillStyle = "red";
        context2.fill();
        CheckIP[1] = 0;
    });

    ros3.on('connection', function() {
        console.log('Robot3 Connected to websocket server.');
        light3 = "connected";
        context3.fillStyle = "green";
        context3.fill();
        CheckIP[2] = 1;
    });
    ros3.on('error', function(error) {
        console.log('Robot3 Error connecting to websocket server:');
        light3 = "disconnected";
        context3.fillStyle = "red";
        context3.fill();
        CheckIP[2] = 0;
    });
    ros3.on('close', function() {
        console.log('Robot3 Connection to websocket server closed.');
        light3 = "disconnected";
        context3.fillStyle = "red";
        context3.fill();
        CheckIP[2] = 0;
    });
    transferRobotNum(RobNum);
}


//Topic
//vector
var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});

var cmdVel2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
var cmdVel3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
//TeamColor
var TeamColor = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
var TeamColor2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
var TeamColor3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
//location
var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/Ros/position',
    messageType: 'geometry_msgs/Twist'
});
var listener2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/Ros2/position',
    messageType: 'geometry_msgs/Twist'
});
var listener3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/Ros3/position',
    messageType: 'geometry_msgs/Twist'
});
//GameState
var GameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
//Role
var Role = new ROSLIB.Topic({
    ros: ros,
    name: '/Role',
    messageType: 'std_msgs/Int32'
});
var Role2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/Role',
    messageType: 'std_msgs/Int32'
});
var Role3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/Role',
    messageType: 'std_msgs/Int32'
});
//IsSimulator
var IsSimulator = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});
var IsSimulator2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});
var IsSimulator3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});
//Vision
var Vision = new ROSLIB.Topic({
    ros: ros,
    name: '/vision/object',
    messageType: '/vision/Object'
});
var Vision2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/vision/object',
    messageType: '/vision/Object'
});
var Vision3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/vision/object',
    messageType: '/vision/Object'
});
//shoot
var TopicShoot = new ROSLIB.Topic({
    ros: ros,
    name: '/shoot',
    messageType: 'std_msgs/Int32'
});
var TopicShoot2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/shoot',
    messageType: 'std_msgs/Int32'
});
var TopicShoot3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/shoot',
    messageType: 'std_msgs/Int32'
});
//ROS_goal_transfer
//topic_message_linear
//topic_GameState
function topicROSGameState(state) {
    var gameState = new ROSLIB.Message({
        data: state
    });
    GameState.publish(gameState);
}

function topicROSGameState2(state) {
    var gameState = new ROSLIB.Message({
        data: state
    });
    GameState2.publish(gameState);
}

function topicROSGameState3(state) {
    var gameState = new ROSLIB.Message({
        data: state
    });
    GameState3.publish(gameState);
}

function topicROSTeamColor(color) {
    console.log(color);
    var teamcolor = new ROSLIB.Message({
        data: color
    });
    TeamColor.publish(teamcolor);
}

function topicROSTeamColor2(color) {
    var teamcolor = new ROSLIB.Message({
        data: color
    });
    TeamColor2.publish(teamcolor);
}

function topicROSTeamColor3(color) {
    var teamcolor = new ROSLIB.Message({
        data: color
    });
    TeamColor3.publish(teamcolor);
}

function topicROStransfer(choice, vec3) {
    // var vector = new vec(0,0,0);

    var twist = new ROSLIB.Message({
        linear: {
            x: vec3.x,
            y: vec3.y,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: vec3.z
        }
    });
    if (choice == 1) {
        cmdVel.publish(twist);
    } else if (choice == 2) {
        cmdVel2.publish(twist);
    } else if (choice == 3) {
        cmdVel3.publish(twist);
    }
}
//position
listener.subscribe(function(message) {
    SavePosition(0, message.linear.x, message.linear.y, message.angular.z);
});
listener2.subscribe(function(message) {
    SavePosition(1, message.linear.x, message.linear.y, message.angular.z);
});
listener3.subscribe(function(message) {
    SavePosition(2, message.linear.x, message.linear.y, message.angular.z);
});

function SavePosition(i, X, Y, Z) {
    if ((RobposX[i] != X) || (RobposY[i] != Y) || (RobposZ[i] != Z)) {
        RobposX[i] = X;
        RobposY[i] = Y;
        RobposZ[i] = Z;
        world_location();
        drawRobot2(RobposX[0], RobposY[0], 22.5, 45, RobposZ[0], 1);
        drawRobot2(RobposX[1], RobposY[1], 22.5, 45, RobposZ[1], 2);
        drawRobot2(RobposX[2], RobposY[2], 22.5, 45, RobposZ[2], 3);
    }
}

//TeamStrategyRole
function TeamStrategyRole(role) {
    var RobStrategy = new ROSLIB.Message({
        data: role[0]
    });
    Role.publish(RobStrategy);
    RobStrategy = new ROSLIB.Message({ data: role[1] });
    Role2.publish(RobStrategy);
    RobStrategy = new ROSLIB.Message({ data: role[2] });
    Role3.publish(RobStrategy);
}
//OpenSimulator
function OpenSimulator(checked) {
    var temp;
    if (checked == true) {
        temp = new ROSLIB.Message({
            data: 1
        });
        IsSimulator.publish(temp);
        temp = new ROSLIB.Message({ data: 1 });
        IsSimulator2.publish(temp);
        temp = new ROSLIB.Message({ data: 1 });
        IsSimulator3.publish(temp);
    } else {
        temp = new ROSLIB.Message({
            data: 0
        });
        IsSimulator.publish(temp);
        temp = new ROSLIB.Message({ data: 0 });
        IsSimulator2.publish(temp);
        temp = new ROSLIB.Message({ data: 0 });
        IsSimulator3.publish(temp);
    }
}

function show_Robot_ip() {
    console.log(' IP: ', document.getElementById("RobotIP").value, ' Port: ', document.getElementById("RobotHost").value);
    console.log(' IP: ', document.getElementById("RobotIP2").value, ' Port: ', document.getElementById("RobotHost2").value);
    console.log(' IP: ', document.getElementById("RobotIP3").value, ' Port: ', document.getElementById("RobotHost3").value);
}

function RobotCloseConnect() {
    ros.close();
    ros2.close();
    ros3.close();
}

//Vision
Vision.subscribe(function(message) {
    SaveVision(0, message.fps, message.ball_dis, message.ball_ang, message.blue_dis, message.blue_ang, message.yellow_dis, message.yellow_ang);
});
Vision2.subscribe(function(message) {
    SaveVision(1, message.fps, message.ball_dis, message.ball_ang, message.blue_dis, message.blue_ang, message.yellow_dis, message.yellow_ang);
});
Vision3.subscribe(function(message) {
    SaveVision(2, message.fps, message.ball_dis, message.ball_ang, message.blue_dis, message.blue_ang, message.yellow_dis, message.yellow_ang);
});

function SaveVision(i, fps, ball_dis, ball_ang, blue_dis, blue_ang, yellow_dis, yellow_ang) {
    if ((VisionBox[i].fps != fps) || (VisionBox[i].ball_dis != ball_dis) || (VisionBox[i].ball_ang != ball_ang) || (VisionBox[i].blue_dis != blue_dis) || (VisionBox[i].blue_ang != blue_ang) || (VisionBox[i].yellow_dis != yellow_dis) || (VisionBox[i].yellow_ang != yellow_ang)) {
        VisionBox[i].fps = fps;
        VisionBox[i].ball_dis = ball_dis;
        VisionBox[i].ball_ang = ball_ang;
        VisionBox[i].blue_dis = blue_dis;
        VisionBox[i].blue_ang = blue_ang;
        VisionBox[i].yellow_dis = yellow_dis;
        VisionBox[i].yellow_ang = yellow_ang;
        world_Vision();
    }
}
//shoot
function ShootSize(size){
    console.log(size);
    var Shoot = new ROSLIB.Message({
        data: size
    });
    TopicShoot.publish(Shoot);
    Shoot = new ROSLIB.Message({data: size});
    TopicShoot2.publish(Shoot);
    Shoot = new ROSLIB.Message({data: size});
    TopicShoot3.publish(Shoot);
}

// parameter
//RobotNumber
var RobotNumber = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/RobotNumber',
});
var RobotNumber2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/RobotNumber',
});
var RobotNumber3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/RobotNumber',
});
// parameter1
//General_variable
var SPlanningVelocityBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SPlanning_Velocity'
});
var DistanceSettingsBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Distance_Settings'
});
//Pathplan_variable
var AttackStrategyBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Attack_Strategy'
});
var ChaseStrategyBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Chase_Strategy'
});
var ZoneAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Zone_Attack'
});
var TypeSAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/TypeS_Attack'
});
var TypeUAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/TypeU_Attack'
});
var SideSpeedUpBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SideSpeedUp'
});
var DorsadAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Dorsad_Attack'
});
var CornerKickBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Corner_Kick'
});
//Behavior_variable
var StateChaseBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Chase_Strategy'
});
var StateAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Attack_Strategy'
});
var StateTypeUChaseBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/TypeU_Chase'
});
var StateTypeSAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/TypeS_Attack'
});
var StateSideSpeedUPBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Side_Speed_UP'
});
var StateZoneAttackBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Zone_Attack'
});
var StateCornerKickBox = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA_Behavior/Corner_Kick'
});
var StrategySelectBox = new ROSLIB.Param({
    ros: ros,
    name: '/StrategySelection',
});
// parameter2
//General_variable
var SPlanningVelocityBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/SPlanning_Velocity'
});
var DistanceSettingsBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Distance_Settings'
});
//Pathplan_variable
var AttackStrategyBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Attack_Strategy'
});
var ChaseStrategyBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Chase_Strategy'
});
var ZoneAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Zone_Attack'
});
var TypeSAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/TypeS_Attack'
});
var TypeUAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/TypeU_Attack'
});
var SideSpeedUpBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/SideSpeedUp'
});
var DorsadAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Dorsad_Attack'
});
var CornerKickBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA/Corner_Kick'
});
//Behavior_variable
var StateChaseBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Chase_Strategy'
});
var StateAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Attack_Strategy'
});
var StateTypeUChaseBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/TypeU_Chase'
});
var StateTypeSAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/TypeS_Attack'
});
var StateSideSpeedUPBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Side_Speed_UP'
});
var StateZoneAttackBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Zone_Attack'
});
var StateCornerKickBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/FIRA_Behavior/Corner_Kick'
});
var StrategySelectBox2 = new ROSLIB.Param({
    ros: ros2,
    name: '/StrategySelection',
});
// parameter3
//General_variable
var SPlanningVelocityBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/SPlanning_Velocity'
});
var DistanceSettingsBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Distance_Settings'
});
//Pathplan_variable
var AttackStrategyBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Attack_Strategy'
});
var ChaseStrategyBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Chase_Strategy'
});
var ZoneAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Zone_Attack'
});
var TypeSAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/TypeS_Attack'
});
var TypeUAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/TypeU_Attack'
});
var SideSpeedUpBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/SideSpeedUp'
});
var DorsadAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Dorsad_Attack'
});
var CornerKickBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA/Corner_Kick'
});
//Behavior_variable
var StateChaseBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Chase_Strategy'
});
var StateAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Attack_Strategy'
});
var StateTypeUChaseBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/TypeU_Chase'
});
var StateTypeSAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/TypeS_Attack'
});
var StateSideSpeedUPBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Side_Speed_UP'
});
var StateZoneAttackBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Zone_Attack'
});
var StateCornerKickBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/FIRA_Behavior/Corner_Kick'
});
var StrategySelectBox3 = new ROSLIB.Param({
    ros: ros3,
    name: '/StrategySelection',
});
//parameter function
//transferRobotNum
function transferRobotNum(robotnumber) {
    /*console.log(robotnumber[2]);*/
    console.log(CheckIP);
    if (CheckIP[0] == 1) {
        console.log(robotnumber);
        RobotNumber.set(robotnumber[0]);
    }
    if (CheckIP[1] == 1) {
        console.log(robotnumber);
        RobotNumber2.set(robotnumber[1]);
    }
    if (CheckIP[2] == 1) {
        console.log(robotnumber);
        RobotNumber3.set(robotnumber[2]);
    }
}


function GeneralTransfer(box1, box2) {
    SPlanningVelocityBox.set(box1);
    DistanceSettingsBox.set(box2);

    SPlanningVelocityBox2.set(box1);
    DistanceSettingsBox2.set(box2);

    SPlanningVelocityBox3.set(box1);
    DistanceSettingsBox3.set(box2);
}

//function GeneralGet() {
var obj;
SPlanningVelocityBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DistanceSettingsBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});

SPlanningVelocityBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DistanceSettingsBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});

SPlanningVelocityBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DistanceSettingsBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
//}

function PathplanTransfer(box1, box2, box3, box4, box5, box6, box7,box8) {
    AttackStrategyBox.set(box1);
    ChaseStrategyBox.set(box2);
    ZoneAttackBox.set(box3);
    TypeSAttackBox.set(box4);
    TypeUAttackBox.set(box5);
    SideSpeedUpBox.set(box6);
    DorsadAttackBox.set(box7);
    CornerKickBox.set(box8);

    AttackStrategyBox2.set(box1);
    ChaseStrategyBox2.set(box2);
    ZoneAttackBox2.set(box3);
    TypeSAttackBox2.set(box4);
    TypeUAttackBox2.set(box5);
    SideSpeedUpBox2.set(box6);
    DorsadAttackBox2.set(box7);
    CornerKickBox2.set(box8);
    console.log(box8);

    AttackStrategyBox3.set(box1);
    ChaseStrategyBox3.set(box2);
    ZoneAttackBox3.set(box3);
    TypeSAttackBox3.set(box4);
    TypeUAttackBox3.set(box5);
    SideSpeedUpBox3.set(box6);
    DorsadAttackBox3.set(box7);
    CornerKickBox3.set(box8);
}
//function PathplanGet() {
var obj;
AttackStrategyBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("AttackStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ChaseStrategyBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ChaseStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ZoneAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeSAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("TypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeUAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("TypeUAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
SideSpeedUpBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SideSpeedUpElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DorsadAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DorsadAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
CornerKickBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("CornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});


AttackStrategyBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("AttackStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ChaseStrategyBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ChaseStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ZoneAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeSAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("TypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeUAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("TypeUAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
SideSpeedUpBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SideSpeedUpElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DorsadAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DorsadAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
CornerKickBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("CornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});

AttackStrategyBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("AttackStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ChaseStrategyBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ChaseStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
ZoneAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("ZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeSAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("TypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
TypeUAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("TypeUAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
SideSpeedUpBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("SideSpeedUpElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
DorsadAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("DorsadAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
CornerKickBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("CornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
//}

function BehaviorTransfer(box1, box2, box3, box4, box5, box6, box7, box8) {
    StateChaseBox.set(box1);
    StateAttackBox.set(box2);
    StateTypeUChaseBox.set(box3);
    StateTypeSAttackBox.set(box4);
    StateSideSpeedUPBox.set(box5);
    StateZoneAttackBox.set(box6);
    StateCornerKickBox.set(box7);
    StrategySelectBox.set(box8);

    StateChaseBox2.set(box1);
    StateAttackBox2.set(box2);
    StateTypeUChaseBox2.set(box3);
    StateTypeSAttackBox2.set(box4);
    StateSideSpeedUPBox2.set(box5);
    StateZoneAttackBox2.set(box6);
    StateCornerKickBox2.set(box7);
    StrategySelectBox2.set(box8);

    StateChaseBox3.set(box1);
    StateAttackBox3.set(box2);
    StateTypeUChaseBox3.set(box3);
    StateTypeSAttackBox3.set(box4);
    StateSideSpeedUPBox3.set(box5);
    StateZoneAttackBox3.set(box6);
    StateCornerKickBox3.set(box7);
    StrategySelectBox3.set(box8);

}
function BehaviorKeyborard(box1) {
    StrategySelectBox.set(box1);
    StrategySelectBox2.set(box1);
    StrategySelectBox3.set(box1);

}
//function BehaviorGet() {
var obj;
StateChaseBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeUChaseBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeUChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeSAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateSideSpeedUPBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateZoneAttackBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateCornerKickBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateCornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StrategySelectBox.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionElement");
        for (var i = 0; i < obj.length; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    }
});

StateChaseBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeUChaseBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeUChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeSAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateSideSpeedUPBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateZoneAttackBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateCornerKickBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateCornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StrategySelectBox2.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionElement");
        for (var i = 0; i < obj.length; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    }
});

StateChaseBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeUChaseBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeUChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateTypeSAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateTypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateSideSpeedUPBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateZoneAttackBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StateCornerKickBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StateCornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});
StrategySelectBox3.get(function(value) {
    if (value != null) {
        obj = document.getElementsByName("StrategySelectionElement");
        for (var i = 0; i < obj.length; i++) {
            if (value[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    }
});
//}

//service
// -----------------

var updateClient = new ROSLIB.Service({
    ros: ros,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});
var updateClient2 = new ROSLIB.Service({
    ros: ros2,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});
var updateClient3 = new ROSLIB.Service({
    ros: ros3,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});

var request = new ROSLIB.ServiceRequest({
    receive: 1
});

function up() {
    document.getElementById("Update").style.cursor = "wait";
    context4.fillStyle = "yellow";
    context4.fill();
    updateClient.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context4.fillStyle = "green";
            context4.fill();
        }
    });

    updateClient2.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context4.fillStyle = "green";
            context4.fill();
        }
    });

    updateClient3.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context4.fillStyle = "green";
            context4.fill();
        }
    });
}
