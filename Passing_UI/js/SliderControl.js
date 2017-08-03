//
function showValue(newValue, name, num) {
    if (newValue > 100) {
        newValue = 100;
    }
    document.getElementsByName(name)[num].value = newValue;
}

function checkvalue(name, newValue, num) {
    if (newValue > 100) {
        newValue = 100;
    }
    document.getElementsByName(name)[num].value = newValue;
}

function rangeValue(newValue, name, num) {
    document.getElementsByName(name).value = newValue;
    0
}

//======================================================================