//===================================================================
//ParameterButton
var TopicParameterButton = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/parameterbutton',
    messageType: '/vision/parameterbutton'
});

function topicROSParameterButton(value) {
    console.log(value);
    if(value == 8)
    {
        value = ButtonFlag;
    }
    var ParameterButton = new ROSLIB.Message({
        button: value
    });
    TopicParameterButton.publish(ParameterButton);
}
//===================================================================
//camera
var TopicCamera = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/camera',
    messageType: '/vision/camera'
});
function topicROSCamera(value) {
    console.log(value);
    value = parseInt(value);
    var Camera = new ROSLIB.Message({
        fps: value
    });
    TopicCamera.publish(Camera);
}
//===================================================================
//center
var TopicCenter = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/center',
    messageType: '/vision/center'
});
var SubTopicCenter = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/CenterDis',
    messageType: '/vision/dis'
});

function topicCenterTransfer() {
    var box = [];

    for (var i = 0; i < 6; i++) {
        box[i] = parseInt(document.getElementsByName('CenterElement')[i].value);
    }
    topicROSCenter(box);
}

function topicROSCenter(box) {
    console.log(box);
    var Center = new ROSLIB.Message({
        CenterX: box[0],
        CenterY: box[1],
        Inner: box[2],
        Outer: box[3],
        Front: box[4],
        Camera_High: box[5]
    });
    TopicCenter.publish(Center);
}
SubTopicCenter.subscribe(function(msg) { 
    console.log(msg.distance);  
    document.getElementsByName('CenterDisLabel')[0].innerText = Math.floor(msg.distance*10)/10;
	
});
//===================================================================
//scan
var TopicScan = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/scan',
    messageType: '/vision/scan'
});

function topicScanTransfer() {
    var box = [];

    for (var i = 0; i < 11; i++) {
        box[i] = parseInt(document.getElementsByName('ScanElement')[i].value);
    }
    topicROSScan(box);
}

function topicROSScan(box) {
    console.log(box);
    var Scan = new ROSLIB.Message({
        Angle_Near_Gap: box[0],
        Magn_Near_Gap: box[1],
        Magn_Near_Start: box[2],
        Magn_Middle_Start: box[3],
        Magn_Far_Start: box[4],
        Magn_Far_End: box[5],
        Dont_Search_Angle_1: box[6],
        Dont_Search_Angle_2: box[7],
        Dont_Search_Angle_3: box[8],
        Angle_range_1: box[9],
        Angle_range_2_3: box[10],
    });
    TopicScan.publish(Scan);
}
//===================================================================
//color
var TopicColor = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/color',
    messageType: '/vision/color'
});
//HSV
function topicColorTransfer() {
    var mode = parseInt(document.getElementById('HSVSelect').value);
    switch (mode) {
        case 0:
            for (var i = 0; i < 6; i++) {
                OrangeBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            break;
        case 1:
            for (var i = 0; i < 6; i++) {
                GreenBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            break;
        case 2:
            for (var i = 0; i < 6; i++) {
                BlueBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
                YellowBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
                WhiteBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            break;
    }
    topicROSColor(mode);
}

function topicROSColor(mode) {
    var Color = new ROSLIB.Message({
        ColorMode: mode,
        BallHSVBox: OrangeBox,
        GreenHSVBox: GreenBox,
        BlueHSVBox: BlueBox,
        YellowHSVBox: YellowBox,
        WhiteHSVBox: WhiteBox,
    });
    TopicColor.publish(Color);
}
//White
var TopicWhite = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/white',
    messageType: '/vision/white'
});
function topicWhiteTransfer() {
    var box = [];
    for (var i = 0; i < 2; i++) {
        box[i] = parseInt(document.getElementsByName('WhiteElement')[i].value);
    }
    topicROSWhite(box);
}

function topicROSWhite(box) {
    console.log(box);
    var White = new ROSLIB.Message({
        Gray: box[0],
        Angle: box[1]
    });
    TopicWhite.publish(White);
}
//Black
var TopicBlack = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/black',
    messageType: '/vision/black'
});
function topicBlackTransfer() {
    var box = [];
    for (var i = 0; i < 2; i++) {
        box[i] = parseInt(document.getElementsByName('BlackElement')[i].value);
    }
    topicROSBlack(box);
}

function topicROSBlack(box) {
    console.log(box);
    var Black = new ROSLIB.Message({
        Gray: box[0],
        Angle: box[1]
    });
    TopicBlack.publish(Black);
}
//colorbutton
var TopicColorButton = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/colorbutton',
    messageType: '/vision/colorbutton'
});
function topicROSColorButton(value) {
    console.log(value);
    var ColorButton = new ROSLIB.Message({
        button: value
    });
    TopicColorButton.publish(ColorButton);
}
//save
var TopicSaveButton = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/bin_save',
    messageType: '/vision/bin'
});
function topicROSSaveButton(value) {
    console.log(64);
    var SaveButton = new ROSLIB.Message({
        bin: 64
    });
    TopicSaveButton.publish(SaveButton);
}
//====================================================================
//checkforparameter
var TopicParameterCheck = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/parametercheck',
    messageType: '/vision/parametercheck'
});

function topicROSCheckButton(value) {
    console.log(value);
    var ParameterCheck = new ROSLIB.Message({
        checkpoint: 64
    });
    TopicParameterCheck.publish(ParameterCheck);
}
//===================================================================
var View = new ROSLIB.Topic({
    ros: ros,
    name: '/vision/view',
    messageType: '/vision/view'
});
function ViewButton(value){
    console.log(value);
    var ViewCheck = new ROSLIB.Message({
        checkpoint: parseInt(value)
    });
    View.publish(ViewCheck);
}
//Monitor
var Monitor = new ROSLIB.Topic({
    ros: ros,
    name: '/vision/object',
    messageType: '/vision/Object'
});
Monitor.subscribe(function(msg) {
    var MBox = [];
    MBox.push(msg.fps);
    MBox.push(msg.ball_x);
    MBox.push(msg.ball_y);
    MBox.push(msg.ball_LR);
    MBox.push(msg.ball_dis);
    MBox.push(msg.ball_ang);

    MBox.push(msg.blue_x);
    MBox.push(msg.blue_y);
    MBox.push(msg.blue_LR);
    MBox.push(msg.blue_dis);
    MBox.push(msg.blue_ang);

    MBox.push(msg.yellow_x);
    MBox.push(msg.yellow_y);
    MBox.push(msg.yellow_LR);
    MBox.push(msg.yellow_dis);
    MBox.push(msg.yellow_ang);
    //console.log(MBox);

      document.getElementsByName('MonitorElement')[0].innerText = MBox[0];
      document.getElementsByName('MonitorElement')[1].innerText = '( '+MBox[1]+','+MBox[2]+' )';
      document.getElementsByName('MonitorElement')[2].innerText = MBox[3];
      document.getElementsByName('MonitorElement')[3].innerText = MBox[4];
      document.getElementsByName('MonitorElement')[4].innerText = MBox[5];

      document.getElementsByName('MonitorElement')[5].innerText = '( '+MBox[6]+','+MBox[7]+' )';
      document.getElementsByName('MonitorElement')[6].innerText = MBox[8];
      document.getElementsByName('MonitorElement')[7].innerText = MBox[9];
      document.getElementsByName('MonitorElement')[8].innerText = MBox[10];

      document.getElementsByName('MonitorElement')[9].innerText = '( '+MBox[11]+','+MBox[12]+' )';
      document.getElementsByName('MonitorElement')[10].innerText = MBox[13];
      document.getElementsByName('MonitorElement')[11].innerText = MBox[14];
      document.getElementsByName('MonitorElement')[12].innerText = MBox[15];
    
});
//====================================================================
//(X,Y)
var TopicPosition = new ROSLIB.Topic({
    ros: ros,
    name: '/interface/position',
    messageType: '/vision/position'
});

function topicROSPosition(x,y) {
    console.log(1111);
    x = parseInt(x);
    y = parseInt(y);
    var position = new ROSLIB.Message({
        PositionX:x,
        PositionY:y
    });
    TopicPosition.publish(position);
}

