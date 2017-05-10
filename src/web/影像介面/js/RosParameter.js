//ParameterCamera
var ParameterCamera = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/FPS',
});

ParameterCamera.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CameraElement");
        obj[0].value = value;
        document.getElementsByName("CameraLabel")[0].innerText = value;
    }
});

function ParameterCameraValue() {
    var value = parseInt(document.getElementsByName('CameraElement')[0].value);
    console.log(value);
    ParameterCamera.set(value);
}
//ParameterCenter_X
var ParameterCenter_X = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Center/Center_X',
});

ParameterCenter_X.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[0].value = value;
        document.getElementsByName("CenterLabel")[0].innerText = value;
    }
});

function ParameterCenterTransfer_X() {
    var value = parseInt(document.getElementsByName('CenterElement')[0].value);
    console.log(value);
    ParameterCenter_X.set(value);
}
//ParameterCenter_Y
var ParameterCenter_Y = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Center/Center_Y',
});

ParameterCenter_Y.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[1].value = value;
        document.getElementsByName("CenterLabel")[1].innerText = value;
    }
});

function ParameterCenterTransfer_Y() {
    var value = parseInt(document.getElementsByName('CenterElement')[1].value);
    console.log(value);
    ParameterCenter_Y.set(value);
}

//ParameterCenter_inner
var ParameterCenter_Inner = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Center/Inner',
});

ParameterCenter_Inner.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[2].value = value;
        document.getElementsByName("CenterLabel")[2].innerText = value;
    }
});

function ParameterCenterTransfer_Inner() {
    var value = parseInt(document.getElementsByName('CenterElement')[2].value);
    console.log(value);
    ParameterCenter_Inner.set(value);
}

//ParameterCenter_Outer
var ParameterCenter_Outer = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Center/Outer',
});

ParameterCenter_Outer.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[3].value = value;
        document.getElementsByName("CenterLabel")[3].innerText = value;
    }
});

function ParameterCenterTransfer_Outer() {
    var value = parseInt(document.getElementsByName('CenterElement')[3].value);
    console.log(value);
    ParameterCenter_Outer.set(value);
}

//ParameterCenter_Front
var ParameterCenter_Front = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Center/Front',
});

ParameterCenter_Front.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[4].value = value;
        document.getElementsByName("CenterLabel")[4].innerText = value;
    }
});

function ParameterCenterTransfer_Front() {
    var value = parseInt(document.getElementsByName('CenterElement')[4].value);
    console.log(value);
    ParameterCenter_Front.set(value);
}
//ParameterCenter_Camera_high
var ParameterCenter_Camera_high = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Center/Camera_high',
});

ParameterCenter_Camera_high.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        obj[5].value = value;
    }
});

function ParameterCenterTransfer_Camera_high() {
    var value = parseInt(document.getElementsByName('CenterElement')[5].value);
    console.log(value);
    ParameterCenter_Camera_high.set(value);
}

//parameterbutton
var Parameterbutton = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/Parameterbutton',
});

function ParameterbuttonTransfer(value) {
    var value;

    console.log(value);
    Parameterbutton.set(value);
}
/*ParameterCenter.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("CenterElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
        }
    }
});*/
//ParameterScan
var ParameterScan = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/scan_para',
});


/*
function ParameterScanTransfer() {
    var box = [];
    $("[name=ScanElement]").each(function() {
        box.push(parseInt($(this).val()));
    });
    console.log(box);
    ParameterScan.set(box);
}*/
//ParameterScan_Angel_Near_Gap
var ParameterScan1 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Angle_Near_Gap',
});

ParameterScan1.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[0].value = value;
        document.getElementsByName("ScanLabel")[0].innerText = value; 
    }
});

function ParameterScanTransfer1() {
    var value = parseInt(document.getElementsByName('ScanElement')[0].value);
    console.log(value);
    ParameterScan1.set(value);
}
//ParameterScan_Magn_Near_Gap
var ParameterScan2 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Magn_Near_Gap',
});

ParameterScan2.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[1].value = value;
        document.getElementsByName("ScanLabel")[1].innerText = value; 
    }
});

function ParameterScanTransfer2() {
    var value = parseInt(document.getElementsByName('ScanElement')[1].value);
    console.log(value);
    ParameterScan2.set(value);
}
//ParameterScan_Magn_Near_Start
var ParameterScan3 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Magn_Near_Start',
});

ParameterScan3.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[2].value = value;
        document.getElementsByName("ScanLabel")[2].innerText = value; 
    }
});

function ParameterScanTransfer3() {
    var value = parseInt(document.getElementsByName('ScanElement')[2].value);
    console.log(value);
    ParameterScan3.set(value);
}
//ParameterScan_Magn_Middel_Start
var ParameterScan4 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Magn_Middle_Start',
});

ParameterScan4.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[3].value = value;
        document.getElementsByName("ScanLabel")[3].innerText = value; 
    }
});

function ParameterScanTransfer4() {
    var value = parseInt(document.getElementsByName('ScanElement')[3].value);
    console.log(value);
    ParameterScan4.set(value);
}
//ParameterScan_Magn_Far_Start
var ParameterScan5 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Magn_Far_Start',
});

ParameterScan5.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[4].value = value;
        document.getElementsByName("ScanLabel")[4].innerText = value; 
    }
});

function ParameterScanTransfer5() {
    var value = parseInt(document.getElementsByName('ScanElement')[4].value);
    console.log(value);
    ParameterScan5.set(value);
}
//ParameterScan_Magn_Far_End
var ParameterScan6 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Magn_Far_End',
});

ParameterScan6.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[5].value = value;
        document.getElementsByName("ScanLabel")[5].innerText = value; 
    }
});


function ParameterScanTransfer6() {
    var value = parseInt(document.getElementsByName('ScanElement')[5].value);
    console.log(value);
    ParameterScan6.set(value);
}
//ParameterScan_Dont_Search_Angle_1
var ParameterScan7 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Dont_Search_Angle_1',
});

ParameterScan7.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[6].value = value;
        document.getElementsByName("ScanLabel")[6].innerText = value; 
    }
});


function ParameterScanTransfer7() {
    var value = parseInt(document.getElementsByName('ScanElement')[6].value);
    console.log(value);
    ParameterScan7.set(value);
}
//ParameterScan_Dont_Search_Angle_2
var ParameterScan8 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Dont_Search_Angle_2',
});

ParameterScan8.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[7].value = value;
        document.getElementsByName("ScanLabel")[7].innerText = value; 
    }
});

function ParameterScanTransfer8() {
    var value = parseInt(document.getElementsByName('ScanElement')[7].value);
    console.log(value);
    ParameterScan8.set(value);
}
//ParameterScan_Dont_Search_Angle_3
var ParameterScan9 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Dont_Search_Angle_3',
});

ParameterScan9.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[8].value = value;
        document.getElementsByName("ScanLabel")[8].innerText = value; 
    }
});

function ParameterScanTransfer9() {
    var value = parseInt(document.getElementsByName('ScanElement')[8].value);
    console.log(value);
    ParameterScan9.set(value);
}
//ParameterScan_Angle_range_1
var ParameterScan10 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Angle_range_1',
});

ParameterScan10.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[9].value = value;
        document.getElementsByName("ScanLabel")[9].innerText = value; 
    }
});

function ParameterScanTransfer10() {
    var value = parseInt(document.getElementsByName('ScanElement')[9].value);
    console.log(value);
    ParameterScan10.set(value);
}
//ParameterScan_Angle_range_2_3
var ParameterScan11 = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN/Angle_range_2_3',
});

ParameterScan11.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("ScanElement");
        obj[10].value = value;
        document.getElementsByName("ScanLabel")[10].innerText = value; 
    }
});


function ParameterScanTransfer11() {
    var value = parseInt(document.getElementsByName('ScanElement')[0].value);
    console.log(value);
    ParameterScan11.set(value);
}
/*
//ParameterScan
var ParameterScan = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/SCAN',
});

function ParameterScanTransfer() {
    var box = [];
    $("[name=ScanElement]").each(function() {
        box.push(parseInt($(this).val()));
    });
    console.log(box);
    ParameterScan.set(box);
}*/
//ParameterHSV
var ParameterHSV_Ball = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/Ball',
});
var ParameterHSV_Green = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/Green',
});
var ParameterHSV_Blue = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/Blue',
});
var ParameterHSV_Yellow = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/Yellow',
});

ParameterHSV_Ball.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = value[i];
            OrangeBox[i] = value[i];
            document.getElementsByName('HSVElement2')[i].value = value[i];
        }
    }
});

ParameterHSV_Green.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            GreenBox[i] = value[i];
        }
    }
});

ParameterHSV_Blue.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            BlueBox[i] = value[i];
        }
    }
});

ParameterHSV_Yellow.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("HSVElement");
        for (var i = 0; i < obj.length; i++) {
            YellowBox[i] = value[i];
        }
    }
});


function ParameterHSVTransfer() {
    var mode = parseInt(document.getElementById('HSVSelect').value);
    var box = [];
    switch (mode) {
        case 0:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            console.log(box);
            ParameterHSV_Ball.set(box);
            break;
        case 1:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            console.log(box);
            ParameterHSV_Green.set(box);
            break;
        case 2:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            console.log(box);
            ParameterHSV_Blue.set(box);
            break;
        case 3:
            for (var i = 0; i < 6; i++) {
                box[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
            }
            console.log(box);
            ParameterHSV_Yellow.set(box);
            break;
        case 4:
            for (var i = 0; i < 6; i++) {
                WhiteBox[i] = parseInt(document.getElementsByName('HSVElement')[i].value);
                //ParameterHSV.set
            }
            break;
    }
}
//ParameterWhite
var ParameterWhite_gray = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/white/gray',
});

ParameterWhite_gray.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("WhiteElement");
        obj[0].value = value;
        document.getElementsByName("WhiteLabel")[0].innerText = value; 
    }
});

function ParameterWhite_grayTransfer() {
    /*var box = [];
    $("[name=WhiteElement]").each(function() {
        box.push(parseInt($(this).val()));
    });
    console.log(box);
    ParameterWhite.set(box);*/
    var value = parseInt(document.getElementsByName('WhiteElement')[0].value);
    console.log(value);
    ParameterWhite_gray.set(value);
}
var ParameterWhite_angle = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/white/angle',
});

ParameterWhite_angle.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("WhiteElement");
        obj[1].value = value;
        document.getElementsByName("WhiteLabel")[1].innerText = value; 
    }
});

function ParameterWhite_angleTransfer() {
    var value = parseInt(document.getElementsByName('WhiteElement')[1].value);
    console.log(value);
    ParameterWhite_angle.set(value);
}
//ParameterBlack
var ParameterBlack_angle = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/black/angle',
});

ParameterBlack_angle.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("BlackElement");
        obj[1].value = value;
        document.getElementsByName("BlackLabel")[1].innerText = value; 
    }
});

function ParameterBlack_angleTransfer() {
    var value = parseInt(document.getElementsByName('BlackElement')[1].value);
    console.log(value);
    ParameterBlack_angle.set(value);
}
var ParameterBlack_gray = new ROSLIB.Param({
    ros: ros,
    name: '/FIRA/HSV/black/gray',
});

ParameterBlack_gray.get(function(value) {
    if (value != null) {
        var obj = document.getElementsByName("BlackElement");
        obj[0].value = value;
        document.getElementsByName("BlackLabel")[0].innerText = value; 
    }
});

function ParameterBlack_grayTransfer() {
    var value = parseInt(document.getElementsByName('BlackElement')[0].value);
    console.log(value);
    ParameterBlack_gray.set(value);
}
