if (typeof(Storage) !== "undefined") {
    var obj;
    // GenenalStorage
    //========================================================================
    if (localStorage.getItem("ParameterStr") != null) {
        obj = document.getElementsByName("ParameterElement");
        for (var i = 0; i < obj.length; i++) {
            obj[i].value = JSON.parse(localStorage.getItem("ParameterStr"))[i];
        }
    } else {
        obj = document.getElementsByName("ParameterElement");
        obj[0].value = 0;
        obj[1].value = 0;
        obj[2].value = 0;
        obj[3].value = 0;
        obj[4].value = 0;
    }
}
