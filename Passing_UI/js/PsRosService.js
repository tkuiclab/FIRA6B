var updateClient = new ROSLIB.Service({
    ros: ros,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});

var request = new ROSLIB.ServiceRequest({
    receive: 1
});

function up() {
    console.log('updating parameter');
    updateClient.callService(request, function(res) {
        if (res.update == 2) {
            console.log('Parameter is saved');
        }
    });
}