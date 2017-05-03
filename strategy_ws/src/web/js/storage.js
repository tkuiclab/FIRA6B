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
    if (localStorage.getItem("GeneralDistanceSetStr") != null) {
        obj = document.getElementsByName("DistanceSettingsElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("GeneralDistanceSetStr"))[i];
        }
    } else {
        obj = document.getElementsByName("DistanceSettingsElement");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
    }
    // PathplanStorage
    //================================================================
    if (localStorage.getItem("PathplanAtkStrategyStr") != null) {
        obj = document.getElementsByName("AttackStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanAtkStrategyStr"))[i];
        }
    } else {
        obj = document.getElementsByName("AttackStrategyElement");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanChaseStrategyStr") != null) {
        obj = document.getElementsByName("ChaseStrategyElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanChaseStrategyStr"))[i];
        }
    } else {
        obj = document.getElementsByName("ChaseStrategyElement");
        obj[0].value = 0;
    }
    if (localStorage.getItem("PathplanZoneAtkStr") != null) {
        obj = document.getElementsByName("ZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanZoneAtkStr"))[i];
        }
    } else {
        obj = document.getElementsByName("ZoneAttackElement");
        obj[0].value = 1.5;
        obj[1].value = 2.2;
    }
    if (localStorage.getItem("PathplanTypeSAtkStr") != null) {
        obj = document.getElementsByName("TypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeSAtkStr"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeSAttackElement");
        obj[0].value = 2.0;
        obj[1].value = 0.2;
    }
    if (localStorage.getItem("PathplanTypeUAtkStr") != null) {
        obj = document.getElementsByName("TypeUAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanTypeUAtkStr"))[i];
        }
    } else {
        obj = document.getElementsByName("TypeUAttackElement");
        obj[0].value = 0.8;
        obj[1].value = 1.0;
        obj[2].value = -90.0;
        obj[3].value = 90.0;
        obj[4].value = 0.6;
        obj[5].value = 0.3;
        obj[6].value = 10.0;
        obj[7].value = 20.0;
    }
    if (localStorage.getItem("PathplanSideSpeedUpStr") != null) {
        obj = document.getElementsByName("SideSpeedUpElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanSideSpeedUpStr"))[i];
        }
    } else {
        obj = document.getElementsByName("SideSpeedUpElement");
        obj[0].value = 0.73;
        obj[1].value = 1.2;
        obj[2].value = 0.7;
        obj[3].value = 5.0;
        obj[4].value = 17.6;
    }
    if (localStorage.getItem("PathplanDorsadAttackStr") != null) {
        obj = document.getElementsByName("DorsadAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanDorsadAttackStr"))[i];
        }
    } else {
        obj = document.getElementsByName("DorsadAttackElement");
        obj[0].value = 8;
        obj[1].value = 90;
        obj[2].value = 180;
        obj[3].value = 2;
        obj[4].value = 1;
        obj[5].value = 2;
    }
    if (localStorage.getItem("PathplanCornerKickStr") != null) {
        obj = document.getElementsByName("CornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("PathplanCornerKickStr"))[i];
        }
    } else {
        obj = document.getElementsByName("CornerKickElement");
        obj[0].value = 1.2;
        obj[1].value = 60;
        obj[2].value = 90;
        obj[3].value = 50;
    }
    // BehaviorStorage
    //================================================================
    if (localStorage.getItem("BehaviorStateChaseStr") != null) {
        obj = document.getElementsByName("StateChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateChaseStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateChaseElement");
        obj[0].value = 5.0;
        obj[1].value = 0.15;
        obj[2].value = 0.2;
        obj[3].value = 16.5;
        obj[4].value = 0.27;
    }
    if (localStorage.getItem("BehaviorStateAtkStr") != null) {
        obj = document.getElementsByName("StateAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateAtkStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateAttackElement");
        obj[0].value = 30.0;
        obj[1].value = 1.0;
        obj[2].value = 1.0;
    }
    if (localStorage.getItem("BehaviorStateTypeUChaseStr") != null) {
        obj = document.getElementsByName("StateTypeUChaseElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeUChaseStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeUChaseElement");
        obj[0].value = 30.0;
        obj[1].value = 0.3;
    }
    if (localStorage.getItem("BehaviorStateTypeSAtkStr") != null) {
        obj = document.getElementsByName("StateTypeSAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateTypeSAtkStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateTypeSAttackElement");
        obj[0].value = 20.0;
        obj[1].value = 1.0;
        obj[2].value = 1.5;
    }
    if (localStorage.getItem("BehaviorStateSideSpeedUPStr") != null) {
        obj = document.getElementsByName("StateSideSpeedUPElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateSideSpeedUPStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateSideSpeedUPElement");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStateZoneAtkStr") != null) {
        obj = document.getElementsByName("StateZoneAttackElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateZoneAtkStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateZoneAttackElement");
        obj[0].value = 0.6;
    }
    if (localStorage.getItem("BehaviorStateCornerKickStr") != null) {
        obj = document.getElementsByName("StateCornerKickElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("BehaviorStateCornerKickStr"))[i];
        }
    } else {
        obj = document.getElementsByName("StateCornerKickElement");
        obj[0].value = 10.0;
        obj[1].value = 0.45;
    }
    if (localStorage.getItem("BehaviorStrategySelectionStr") != null) {
        obj = document.getElementsByName("StrategySelectionElement");
        for (var i = 0; i < obj.length; i++) {
            if (JSON.parse(localStorage.getItem("BehaviorStrategySelectionStr"))[i] == 1) {
                obj[i].checked = true;
            } else {
                obj[i].checked = false;
            }
        }
    } else {
        obj = document.getElementsByName("StrategySelectionElement");
        obj[0].checked = true;
        obj[1].checked = false;
        obj[2].checked = true;
        obj[3].checked = false;
        obj[4].checked = false;
        obj[5].checked = false;
    }
}
