/*========================================================*/
//GameState
var GameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});

function PublishTopicGameState(state) {
    console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    if (CheckIP[0] == 1)
        GameState1.publish(gameState);
}
/*========================================================*/
//TeamColor
var TeamColor1 = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
function PublishTopicTeamColor(color) {
    var teamcolor = new ROSLIB.Message({
        data: color
    });

    if (CheckIP[0] == 1)
        TeamColor1.publish(teamcolor);
}
/*========================================================*/
//IsSimulator
var IsSimulator1 = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/IsSimulator',
    messageType: 'std_msgs/Int32'
});


setTimeout(PublishTopicSimulator, 0,false);
setTimeout(PublishTopicSimulator, 100,false);
setTimeout(PublishTopicSimulator, 200,false);

function PublishTopicSimulator(checked) {
    var temp;
    if (checked == true) {
        temp = new ROSLIB.Message({
            data: 1
        });
        if (CheckIP[0] == 1)
            IsSimulator1.publish(temp);
    } else {
        temp = new ROSLIB.Message({
            data: 0
        });
        if (CheckIP[0] == 1)
            IsSimulator1.publish(temp);
    }
}
/*========================================================*/
//vector
var cmdVel1 = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
function StrategyStop() {
    setTimeout(StandBy, 0);
    setTimeout(StandBy, 100);
    setTimeout(StandBy, 200);
    setTimeout(StandBy, 300);
    setTimeout(StandBy, 400);
}

function StandBy() {
    var twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    console.log(twist);
    cmdVel1.publish(twist);
}

function PublishTopicCmdVel(vec3) {
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
    if (ChooseRobot == 1) {
        cmdVel1.publish(twist);
    }
}

/*========================================================*/
//shoot
var TopicShoot1 = new ROSLIB.Topic({
    ros: ros,
    name: '/shoot',
    messageType: 'std_msgs/Int32'
});

//shoot
function PublishTopicShoot(size) {
    console.log(size);
    var Shoot = new ROSLIB.Message({
        data: size
    });

    if (ChooseRobot == 1) {
        TopicShoot1.publish(Shoot);}

}
/*========================================================*/
//Vision
var Vision1 = new ROSLIB.Topic({
    ros: ros,
    name: '/vision/object',
    messageType: '/vision/Object'
});
//Vision
Vision1.subscribe(function(msg) {
    var VBox = [];
    VBox.push(msg.fps);
    VBox.push(msg.ball_dis);
    VBox.push(msg.ball_ang);
    VBox.push(msg.blue_dis);
    VBox.push(msg.blue_ang);
    VBox.push(msg.yellow_dis);
    VBox.push(msg.yellow_ang);
    SaveVision(0, VBox);
});

function SaveVision(i, VBox) {
    if ((VisionBox[i].fps != VBox[0]) || (VisionBox[i].ball_dis != VBox[1]) ||
        (VisionBox[i].ball_ang != VBox[2]) || (VisionBox[i].blue_dis != VBox[3]) ||
        (VisionBox[i].blue_ang != VBox[4]) || (VisionBox[i].yellow_dis != VBox[5]) ||
        (VisionBox[i].yellow_ang != VBox[6])) {

        VisionBox[i].fps = VBox[0];
        VisionBox[i].ball_dis = VBox[1];
        VisionBox[i].ball_ang = VBox[2];
        VisionBox[i].blue_dis = VBox[3];
        VisionBox[i].blue_ang = VBox[4];
        VisionBox[i].yellow_dis = VBox[5];
        VisionBox[i].yellow_ang = VBox[6];

        document.getElementsByName('RobotVision' + (i + 1))[0].innerText = VBox[0];
        document.getElementsByName('RobotVision' + (i + 1))[1].innerText = '( ' + VBox[1] + ',' + VBox[2] + ' )';
        document.getElementsByName('RobotVision' + (i + 1))[2].innerText = '( ' + VBox[3] + ',' + VBox[4] + ' )';
        document.getElementsByName('RobotVision' + (i + 1))[3].innerText = '( ' + VBox[5] + ',' + VBox[6] + ' )';
    }
}
/*========================================================*/
//TeamStrategyInformation
//pub
//ros1
var TSInfoPub12 = new ROSLIB.Topic({
    ros: ros2,
    name: '/r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub13 = new ROSLIB.Topic({
    ros: ros3,
    name: '/r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//ros2
var TSInfoPub21 = new ROSLIB.Topic({
    ros: ros,
    name: '/r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub23 = new ROSLIB.Topic({
    ros: ros3,
    name: '/r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//ros3
var TSInfoPub31 = new ROSLIB.Topic({
    ros: ros,
    name: '/r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub32 = new ROSLIB.Topic({
    ros: ros2,
    name: '/r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//subscribe
var TSInfoListen1 = new ROSLIB.Topic({
    ros: ros,
    name: '/r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoListen2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoListen3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});

TSInfoListen1.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 3; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
    info = new ROSLIB.Message({
        data: Box
    });
    if (CheckIP[1])
        TSInfoPub12.publish(info);
    if (CheckIP[2])
        TSInfoPub13.publish(info);
});

TSInfoListen2.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 3; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
    info = new ROSLIB.Message({
        data: Box
    });

    if (CheckIP[0])
        TSInfoPub21.publish(info);
    if (CheckIP[2]) {
        TSInfoPub23.publish(info);
    }
});

TSInfoListen3.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 3; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
    info = new ROSLIB.Message({
        data: Box
    });

    if (CheckIP[0])
        TSInfoPub31.publish(info);
    if (CheckIP[1]) {
        TSInfoPub32.publish(info);
    }
});