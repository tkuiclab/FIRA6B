var worldmap_ctx = worldmap_canvas.getContext("2d");
var window_scale;

worldmap_img.onload = function() {
    Str_Area();
}
worldmap_img.src = 'img/PsGround.jpg';

/*function LevelStrArea(level){
	if(level == 1){
		worldmap_img.src = 'img/PsGround.jpg';
		Str_Area();
	}else if(level == 2){
		worldmap_img.src = 'img/PsGround2.jpg';
		Str_Area();
	}else if(level == 3){
		worldmap_img.src = 'img/PsGround3.jpg';
		Str_Area();
	}else if(level == 4){
		worldmap_img.src = 'img/PsGround4.jpg';
		Str_Area();
	}
}*/

function Str_Area() {
    windowWidth = window.innerWidth;
    windowHeight = window.innerHeight;
    windowWidthToHeight = windowWidth / windowHeight;

    if (windowWidthToHeight > widthToheight) {
        windowWidth = windowHeight * widthToheight;
        strategy_Area.style.height = windowHeight + 'px';
        strategy_Area.style.width = windowWidth + 'px';
    } else {
        windowHeight = windowWidth / widthToheight;
        strategy_Area.style.width = windowWidth + 'px';
        strategy_Area.style.height = windowHeight + 'px';
    }

    window_scale = windowWidth / 750;

    strategy_Area.style.marginTop = (-windowHeight / 2) + 'px';
    strategy_Area.style.marginLeft = (-windowWidth / 2) + 'px';

    //worldmap_canvas.width = windowWidth;
    //worldmap_canvas.height = windowWidth/worldmap.widthToheight;
    //worldmap_ctx.drawImage(worldmap_img, 0, 0 ,windowWidth,windowWidth/worldmap.widthToheight);

    /*	function joystick_main(){
    		var ctx = joystick_Area.getContext("2d");
    		var cty = joystick_Area.getContext("2d");

    		ctx.moveTo1(75,75);
    		ctx.lineTo(150,75);
    		ctx.strokeStyle="#0000ff";
    	};*/
    //joystick_main();
    // drawRobot();
    //world_location(23.5,23.5);
}

window.addEventListener('resize', Str_Area, false);
window.addEventListener('orientationchange', Str_Area, false);

/*worldmap_canvas.addEventListener("mousedown", function(e){
	var pos = getMousePos(worldmap_canvas, e);
	//alert("x:" + pos.x + " y:" + pos.y);
	console.log(worldmap_canvas.offsetWidth,worldmap_canvas.offsetHeight);
	console.log(pos.x ,pos.y);
	drawRobot2(pos.x,pos.y,22.5,90,0);
})*/
/*worldmap_canvas.addEventListener("mouseup", function(e){
	worldmap_ctx.clearRect(0,0,windowWidth,windowHeight);
	Str_Area();
})*/
var Robot_1 = {
    x: 23.5,
    y: 23.5,
    r: 22.5,
    R: 45,
    rad: 0
};

/*function world_location(){
	worldmap_ctx.clearRect(0,0,windowWidth,windowHeight);
	Str_Area();
	var x = 10 * window_scale;
	var y = 500 * window_scale;
	
	worldmap_ctx.textAlign = "start"; 
	worldmap_ctx.font = 15*window_scale+"px Lucida Console";
	worldmap_ctx.fillStyle = "black";
	worldmap_ctx.fillText("Robot X : "+Robpos[0]+" Y : "+Robpos[1]+" Angle : "+Robpos[2], x, y);
}*/
function world_Location() {
    var canva = document.getElementById("Rect1");
    var ctx = canva.getContext("2d");
    ctx.clearRect(0, 0, 1200, 200);

    //var x = 10 * window_scale;
    //var y = 500 * window_scale;

    ctx.textAlign = "start";
    ctx.font = 15 * window_scale + "px Lucida Console";
    ctx.fillStyle = "rgb(255,255,255)";
    ctx.fillText("position X :  " + Robpos[0] + " Y : " + Robpos[1] + " Angle : " + Robpos[2], 30, 120);
    ctx.fillText("cmd_vel  X :  " + Robcmdvel[0] + " Y : " + Robcmdvel[1] + " Angle : " + Robcmdvel[2], 30, 150);
    ctx.fillText("shoot      :  " + Robshoot,30,180);
}
/*function drawRobot(){
	var x = Robot_1.x*window_scale;
	var y = Robot_1.y*window_scale;
	var r = Robot_1.r*window_scale;

	worldmap_ctx.beginPath();
	worldmap_ctx.strokeStyle = "black";
	worldmap_ctx.arc(x,y,r,0,2*Math.PI);
	worldmap_ctx.lineWidth = 1;
	worldmap_ctx.stroke();
	worldmap_ctx.fillStyle = "red";
	worldmap_ctx.globalAlpha=0.7;
	worldmap_ctx.fill();
	
	worldmap_ctx.beginPath();
	worldmap_ctx.strokeStyle = "blue";
	worldmap_ctx.moveTo(x,y);
	worldmap_ctx.lineTo(x+r*Math.cos(Robot_1.rad),y+r*Math.sin(Robot_1.rad));
	worldmap_ctx.stroke();
	
}*/

function drawRobot2(x, y, r, R, rad, number) {
    x = (x + 375) * window_scale;
    y = (y + 275) * window_scale;
    r = r * window_scale;

    // x = x+375;
    // y = y+275;

    worldmap_ctx.beginPath();
    worldmap_ctx.strokeStyle = "black";
    worldmap_ctx.arc(x, y, r, 0, 2 * Math.PI);
    worldmap_ctx.lineWidth = 1;
    worldmap_ctx.stroke();
    worldmap_ctx.fillStyle = "red";
    worldmap_ctx.globalAlpha = 0.7;
    worldmap_ctx.fill();

    worldmap_ctx.beginPath();
    worldmap_ctx.strokeStyle = "blue";
    worldmap_ctx.moveTo(x, y);
    worldmap_ctx.lineTo(x + r * Math.cos(rad), y + r * Math.sin(rad));
    worldmap_ctx.stroke();
    worldmap_ctx.strokeText(number, x, y);
}

function Level0DrawLine() {
    document.getElementById("myImg1").style.visibility = 'hidden';
    document.getElementById("myImg2").style.visibility = 'hidden';
    document.getElementById("myImg3").style.visibility = 'hidden';
    document.getElementById("myImg").src = 'img/PsGround.jpg';
    document.getElementById("myImg").style.visibility = 'visible';
    document.getElementById("myImg").style.opacity = 1;
    Str_Area();
}

function Level1DrawLine() {
    document.getElementById("myImg1").style.visibility = 'hidden';
    document.getElementById("myImg2").style.visibility = 'hidden';
    document.getElementById("myImg3").style.visibility = 'hidden';
    document.getElementById("myImg").src = 'img/6.jpg';
    document.getElementById("myImg").style.visibility = 'visible';
    document.getElementById("myImg").style.opacity = 1;
    Str_Area();
    /* var obj = document.getElementsByName("Level1TargetElement2");
     for (var i = 0; i < obj.length; i++) {
         if (obj[i].checked == true) {
             if (obj[i].value == 0) {
                 document.getElementById("myImg").src = 'img/5.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
                 Str_Area();
             } else if (obj[i].value == 1) {
                 document.getElementById("myImg").src = 'img/6.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
                 Str_Area();
             } else if (obj[i].value == 2) {
                 document.getElementById("myImg").src = 'img/7.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
                 Str_Area();
             } else if (obj[i].value == 3) {
                 document.getElementById("myImg").src = 'img/8.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
                 Str_Area();
             }
         }
     }*/
}

function Level2DrawLine() {
    document.getElementById("myImg2").style.visibility = 'hidden';
    document.getElementById("myImg3").style.visibility = 'hidden';
    document.getElementById("myImg").src = 'img/1.jpg';
    document.getElementById("myImg").style.visibility = 'visible';
    document.getElementById("myImg").style.opacity = 1;
    document.getElementById("myImg1").src = 'img/16.jpg';
    document.getElementById("myImg1").style.visibility = 'visible';
    document.getElementById("myImg1").style.opacity = 0.5;
    Str_Area();
    /* var obj = document.getElementsByName("Level2TargetElement");
     for (var i = 0; i < obj.length; i++) {
         if (obj[i].checked == true) {
             if (obj[i].value == 0) {
                 document.getElementById("myImg").src = 'img/1.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             } else if (obj[i].value == 1) {
                 document.getElementById("myImg").src = 'img/2.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             } else if (obj[i].value == 2) {
                 document.getElementById("myImg").src = 'img/3.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             } else if (obj[i].value == 3) {
                 document.getElementById("myImg").src = 'img/4.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             }
         }
     }
     var obj = document.getElementsByName("Level2TargetElement4");
     for (var i = 0; i < obj.length; i++) {
         if (obj[i].checked == true) {
             if (obj[i].value == 0) {
                 document.getElementById("myImg1").src = 'img/13.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.5;
                 Str_Area();
             } else if (obj[i].value == 1) {
                 document.getElementById("myImg1").src = 'img/14.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.5;
                 Str_Area();
             } else if (obj[i].value == 2) {
                 document.getElementById("myImg1").src = 'img/15.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.5;
                 Str_Area();
             } else if (obj[i].value == 3) {
                 document.getElementById("myImg1").src = 'img/16.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.5;
                 Str_Area();
             }
         }
     }*/
}

function Level3DrawLine() {
    document.getElementById("myImg3").style.visibility = 'hidden';
    document.getElementById("myImg").src = 'img/4.jpg';
    document.getElementById("myImg").style.visibility = 'visible';
    document.getElementById("myImg").style.opacity = 1;
    document.getElementById("myImg1").src = 'img/6.jpg';
    document.getElementById("myImg1").style.visibility = 'visible';
    document.getElementById("myImg1").style.opacity = 0.6;
    document.getElementById("myImg2").src = 'img/13.jpg';
    document.getElementById("myImg2").style.visibility = 'visible';
    document.getElementById("myImg2").style.opacity = 0.4;
    Str_Area();
    /* var obj = document.getElementsByName("Level3TargetElement");
     for (var i = 0; i < obj.length; i++) {
         if (obj[i].checked == true) {
             if (obj[i].value == 0) {
                 document.getElementById("myImg").src = 'img/1.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             } else if (obj[i].value == 1) {
                 document.getElementById("myImg").src = 'img/2.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             } else if (obj[i].value == 2) {
                 document.getElementById("myImg").src = 'img/3.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             } else if (obj[i].value == 3) {
                 document.getElementById("myImg").src = 'img/4.jpg';
                 document.getElementById("myImg").style.visibility = 'visible';
                 document.getElementById("myImg").style.opacity = 1;
             }
         }
     }
     obj = document.getElementsByName("Level3TargetElement2");
     for (var i = 0; i < obj.length; i++) {
         if (obj[i].checked == true) {
             if (obj[i].value == 0) {
                 document.getElementById("myImg1").src = 'img/5.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.6;
             } else if (obj[i].value == 1) {
                 document.getElementById("myImg1").src = 'img/6.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.6;
             } else if (obj[i].value == 2) {
                 document.getElementById("myImg1").src = 'img/7.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.6;
             } else if (obj[i].value == 3) {
                 document.getElementById("myImg1").src = 'img/8.jpg';
                 document.getElementById("myImg1").style.visibility = 'visible';
                 document.getElementById("myImg1").style.opacity = 0.6;
             }
         }
     }
     obj = document.getElementsByName("Level3TargetElement4");
     for (var i = 0; i < obj.length; i++) {
         if (obj[i].checked == true) {
             if (obj[i].value == 0) {
                 document.getElementById("myImg2").src = 'img/13.jpg';
                 document.getElementById("myImg2").style.visibility = 'visible';
                 document.getElementById("myImg2").style.opacity = 0.4;
                 Str_Area();
             } else if (obj[i].value == 1) {
                 document.getElementById("myImg2").src = 'img/14.jpg';
                 document.getElementById("myImg2").style.visibility = 'visible';
                 document.getElementById("myImg2").style.opacity = 0.4;
                 Str_Area();
             } else if (obj[i].value == 2) {
                 document.getElementById("myImg2").src = 'img/15.jpg';
                 document.getElementById("myImg2").style.visibility = 'visible';
                 document.getElementById("myImg2").style.opacity = 0.4;
                 Str_Area();
             } else if (obj[i].value == 3) {
                 document.getElementById("myImg2").src = 'img/16.jpg';
                 document.getElementById("myImg2").style.visibility = 'visible';
                 document.getElementById("myImg2").style.opacity = 0.4;
                 Str_Area();
             }
         }
     }*/
}

function Level4DrawLine() {
    document.getElementById("myImg").src = 'img/3.jpg';
    document.getElementById("myImg").style.visibility = 'visible';
    document.getElementById("myImg").style.opacity = 1;
    document.getElementById("myImg1").src = 'img/8.jpg';
    document.getElementById("myImg1").style.visibility = 'visible';
    document.getElementById("myImg1").style.opacity = 0.5;
    document.getElementById("myImg2").src = 'img/9.jpg';
    document.getElementById("myImg2").style.visibility = 'visible';
    document.getElementById("myImg2").style.opacity = 0.4;
    document.getElementById("myImg3").src = 'img/14.jpg';
    document.getElementById("myImg3").style.visibility = 'visible';
    document.getElementById("myImg3").style.opacity = 0.2;
    Str_Area();
    /*var obj = document.getElementsByName("Level4TargetElement");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked == true) {
            if (obj[i].value == 0) {
                document.getElementById("myImg").src = 'img/1.jpg';
                document.getElementById("myImg").style.visibility = 'visible';
                document.getElementById("myImg").style.opacity = 1;
            } else if (obj[i].value == 1) {
                document.getElementById("myImg").src = 'img/2.jpg';
                document.getElementById("myImg").style.visibility = 'visible';
                document.getElementById("myImg").style.opacity = 1;
            } else if (obj[i].value == 2) {
                document.getElementById("myImg").src = 'img/3.jpg';
                document.getElementById("myImg").style.visibility = 'visible';
                document.getElementById("myImg").style.opacity = 1;
            } else if (obj[i].value == 3) {
                document.getElementById("myImg").src = 'img/4.jpg';
                document.getElementById("myImg").style.visibility = 'visible';
                document.getElementById("myImg").style.opacity = 1;
            }
        }
    }
    obj = document.getElementsByName("Level4TargetElement2");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked == true) {
            if (obj[i].value == 0) {
                document.getElementById("myImg1").src = 'img/5.jpg';
                document.getElementById("myImg1").style.visibility = 'visible';
                document.getElementById("myImg1").style.opacity = 0.5;
            } else if (obj[i].value == 1) {
                document.getElementById("myImg1").src = 'img/6.jpg';
                document.getElementById("myImg1").style.visibility = 'visible';
                document.getElementById("myImg1").style.opacity = 0.5;
            } else if (obj[i].value == 2) {
                document.getElementById("myImg1").src = 'img/7.jpg';
                document.getElementById("myImg1").style.visibility = 'visible';
                document.getElementById("myImg1").style.opacity = 0.5;
            } else if (obj[i].value == 3) {
                document.getElementById("myImg1").src = 'img/8.jpg';
                document.getElementById("myImg1").style.visibility = 'visible';
                document.getElementById("myImg1").style.opacity = 0.5;
            }
        }
    }
    obj = document.getElementsByName("Level4TargetElement3");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked == true) {
            if (obj[i].value == 0) {
                document.getElementById("myImg2").src = 'img/9.jpg';
                document.getElementById("myImg2").style.visibility = 'visible';
                document.getElementById("myImg2").style.opacity = 0.4;
            } else if (obj[i].value == 1) {
                document.getElementById("myImg2").src = 'img/10.jpg';
                document.getElementById("myImg2").style.visibility = 'visible';
                document.getElementById("myImg2").style.opacity = 0.4;
            } else if (obj[i].value == 2) {
                document.getElementById("myImg2").src = 'img/11.jpg';
                document.getElementById("myImg2").style.visibility = 'visible';
                document.getElementById("myImg2").style.opacity = 0.4;
            } else if (obj[i].value == 3) {
                document.getElementById("myImg2").src = 'img/12.jpg';
                document.getElementById("myImg2").style.visibility = 'visible';
                document.getElementById("myImg2").style.opacity = 0.4;
            }
        }
    }
    obj = document.getElementsByName("Level4TargetElement4");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked == true) {
            if (obj[i].value == 0) {
                document.getElementById("myImg3").src = 'img/13.jpg';
                document.getElementById("myImg3").style.visibility = 'visible';
                document.getElementById("myImg3").style.opacity = 0.2;
                Str_Area();
            } else if (obj[i].value == 1) {
                document.getElementById("myImg3").src = 'img/14.jpg';
                document.getElementById("myImg3").style.visibility = 'visible';
                document.getElementById("myImg3").style.opacity = 0.2;
                Str_Area();
            } else if (obj[i].value == 2) {
                document.getElementById("myImg3").src = 'img/15.jpg';
                document.getElementById("myImg3").style.visibility = 'visible';
                document.getElementById("myImg3").style.opacity = 0.2;
                Str_Area();
            } else if (obj[i].value == 3) {
                document.getElementById("myImg3").src = 'img/16.jpg';
                document.getElementById("myImg3").style.visibility = 'visible';
                document.getElementById("myImg3").style.opacity = 0.2;
                Str_Area();
            }
        }
    }*/
}
/*var canvas = document.getElementById('TargetDirection');
var ctx = canvas.getContext('2d');*/

/*function grid() {
    for (var x = 0.5; x < canvas.width; x += 10) {
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
    }
    for (var y = 0.5; y < canvas.height; y += 10) {
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
    }
    ctx.strokeStyle = "RGBA(0,0,0,0)";
    ctx.stroke();
}

function arrow(p1, p2) {

    ctx.save();
    var dist = Math.sqrt((p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1]));

    ctx.beginPath();
    ctx.lineWidth = 5;
    ctx.strokeStyle = "rgb(128,255,255)";
    ctx.moveTo(p1[0], p1[1]);
    ctx.lineTo(p2[0], p2[1]);
    ctx.stroke();

    var angle = Math.acos((p2[1] - p1[1]) / dist);

    if (p2[0] < p1[0]) angle = 2 * Math.PI - angle;

    var size = 15;

    ctx.beginPath();
    ctx.translate(p2[0], p2[1]);
    ctx.rotate(-angle);
    ctx.fillStyle = "rgb(128,255,255)";
    ctx.lineWidth = 7;
	ctx.strokeStyle = "rgb(128,255,255)";  // 把「填滿樣式」設為紅 200 綠 0 藍 0rgb(255, 26, 198)
    ctx.moveTo(0, -size);
    ctx.lineTo(-size, -size);
    ctx.lineTo(0, 0);
    ctx.lineTo(size, -size);
    ctx.lineTo(0, -size);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
}

$('#TargetDirection').bind('mousedown', function (e) {
    var offset = $(this).offset();
    var p1 = [70.0, 540.0];
    var p2 = [e.pageX - offset.left, e.pageY - offset.top];

    ctx.clearRect(0, 0, canvas.width,canvas.height);
    grid();
    arrow(p1, p2);
})

window.addEventListener('resize', resizeGame, false);
window.addEventListener('orientationchange', resizeGame, false);

function resizeGame() {
    var gameArea = document.getElementById('LineArea');
    var widthToHeight = 12 / 20;
    var newWidth = window.innerWidth;12
    var newHeight = window.innerHeight;
    var newWidthToHeight = newWidth / newHeight;

    if (newWidthToHeight > widthToHeight) {
        newWidth = newHeight * widthToHeight;
        gameArea.style.height = newHeight + 'px';
        gameArea.style.width = newWidth + 'px';
    } else {
        newHeight = newWidth / widthToHeight;
        gameArea.style.width = newWidth + 'px';
        gameArea.style.height = newHeight + 'px';
    }
    
    gameArea.style.marginTop = (-newHeight / 2) + 'px';
    gameArea.style.marginLeft = (-newWidth / 2) + 'px';
    
    var gameCanvas = document.getElementById('TargetDirection');
    gameCanvas.width = newWidth;
    gameCanvas.height = newHeight;
}*/
