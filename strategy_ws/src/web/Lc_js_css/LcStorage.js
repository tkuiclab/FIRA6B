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
        obj[2].value = 80.0;
        obj[3].value = 50.0;
        obj[4].value = 20.0;
        obj[5].value = 3.0;
        obj[6].value = 144.0;
        obj[7].value = 5.0;
    }
    /*if (localStorage.getItem("GeneralDistanceSetStr") != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralDistanceSetStr"))[i];
        }
    } else {
        obj = document.getElementsByName("DistanceSettingsElement");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
    }*/
}
  