Element.prototype.leftTopScreen = function() {
    var x = this.offsetLeft;
    var y = this.offsetTop;
    var element = this.offsetParent;

    while (element !== null) {
        x = parseInt(x) + parseInt(element.offsetLeft);
        y = parseInt(y) + parseInt(element.offsetTop);

        element = element.offsetParent;
    }
    return new Array(x, y);
}

Localization_canvas.addEventListener("mousedown", function(e) {

    var xy = Localization_canvas.leftTopScreen();

    //context.fillStyle = 'rgba(255, 255, 255, 0)';
    //context.fillRect(0, 0, 659, 493);

    Localization_canvas.addEventListener("click", function(event) {
        var x = event.clientX;
        var y = event.clientY;
        var xd = x - xy[0];
        var yd = y - xy[1];
        var pose = { x: 0, y: 0 };
        if (xd > centerX)
            pose.x = xd - centerX - ((xd - centerX) / ((300 + errorX) / errorX));
        else if (xd == centerX)
            pose.x = xd - centerX;
        else
            pose.x = xd - centerX - ((xd - centerX) / ((300 + errorX) / errorX));

        if (yd > centerY)
            pose.y = -(yd - centerY) + ((yd - centerY) / ((200 + errorY) / errorY));
        else if (yd == centerY)
            pose.y = yd - centerY;
        else
            pose.y = -(yd - centerY) + ((yd - centerY) / ((200 + errorY) / errorY));

        Draw_Robot(pose.x, pose.y, 0);
        console.log(x - xy[0], y - xy[1], pose.x, pose.y);
        Reset_Localization(parseFloat(pose.x),parseFloat(pose.y));
    });
});

function Draw_Robot(x, y, front) {

    Localization_ctx.clearRect((xt - 22) * CtxToFact_width, (yt - 22) * CtxToFact_height, 20, 20);

    x = x + centerX;
    y = (-y) + centerY;

    if (x > 0)
        x += ((x - centerX) / ((300 + errorX) / errorX));
    else if (x == 0)
        x = 0;
    else
        x -= ((x - centerX) / ((300 + errorX) / errorX));

    if (y > 0)
        y += ((y - centerY) / ((200 + errorY) / errorY));
    else if (yd == 0)
        y = 0;
    else
        y += ((y - centerY) / ((200 + errorY) / errorY));

    Localization_ctx.beginPath();
    Localization_ctx.arc(x * CtxToFact_width, y * CtxToFact_height, 20 * CtxToFact_height, 0, 2 * Math.PI);
    Localization_ctx.fillStyle = '#FFF';
    Localization_ctx.globalAlpha = 0.7;
    Localization_ctx.fill();

    Localization_ctx.beginPath();
    Localization_ctx.lineWidth = 0.4;
    Localization_ctx.moveTo(x * CtxToFact_width, y * CtxToFact_height);
    Localization_ctx.lineTo(x * CtxToFact_width + (20 * CtxToFact_height * Math.cos(front)), y * CtxToFact_height + (20 * CtxToFact_height * Math.sin(front)));
    Localization_ctx.strokeStyle = 'red';
    Localization_ctx.stroke();

    xt = x;
    yt = y;

}