if (typeof(Storage) !== "undefined") {
    var obj;
    // GenenalStorage
    //========================================================================
    if (localStorage.getItem("GeneralSPlanStr") != null) {
        obj = document.getElementsByName("SPlanningVelocityElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralSPlanStr"))[i];
        }
    } else {
        obj = document.getElementsByName("SPlanningVelocityElement");
        obj[0].value = 2.2;
        obj[1].value = 0.3;
        obj[2].value = 15.0;
        obj[3].value = 10.0;
        obj[4].value = 3.0;
        obj[5].value = 3.0;
        obj[6].value = 144.0;
        obj[7].value = 5.0;
    }
    if (localStorage.getItem("GeneralDistantStr") != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralDistantStr"))[i];
        }
    } else {
        obj = document.getElementsByName("DistanceSettingsElement");
        obj[0].value = 51;
        obj[1].value = 90;
        obj[2].value = 53;
        obj[3].value = 100;
        obj[4].value = 110;
        obj[5].value = 51;
        obj[6].value = 6;
    }
    if (localStorage.getItem("GeneralScanLineStr") != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralScanLineStr"))[i];
        }
    } else {
        obj = document.getElementsByName("DistanceSettingsElement");
        obj[0].value = 18;
        obj[1].value = 5;
        obj[2].value = 6;
        obj[3].value = 6;
        obj[4].value = 5;
        obj[5].value = 5;
        obj[6].value = 8;
    }
    if (localStorage.getItem("GeneralChooseSideStr") != null) {
        obj = document.getElementsByName("ChooseSideElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralChooseSideStr"))[i];
        }
    } else {
        obj = document.getElementsByName("ChooseSideElement");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0.03;
        obj[2].value = 0.04;
    }
}
  