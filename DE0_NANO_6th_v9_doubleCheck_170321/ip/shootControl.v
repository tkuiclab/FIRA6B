`default_nettype  none
module ShootControl (
					iClk,
					iRst_n,
					iPower,
					oPower,
);
input			iClk;
input			iRst_n;
input	[7:0]	iPower;
output			oPower;

reg		[15:0]	rCount_1ms;
reg				rClk_1ms;
reg		[7:0]	rShoot_power;
reg				rShoot;

wire			wClk_1ms;
always@(posedge iClk)begin	// 50MHz -> 1khz(1ms)
	if(!iRst_n)begin
		rCount_1ms <= 0;
		rClk_1ms <= 0;
	end
	else if(rCount_1ms < 25000) begin
		rCount_1ms <= rCount_1ms+1;
	end
	else begin
		rCount_1ms <= 0;
		rClk_1ms <= ~rClk_1ms;
	end
end
assign wClk_1ms = rClk_1ms;
always@(posedge wClk_1ms or negedge iRst_n)begin	// control shoot power
	if(!iRst_n)begin
		rShoot_power <= 0;
		rShoot <= 0;
	end
	else if(rShoot_power < iPower) begin
		rShoot_power <= rShoot_power+1;
		rShoot <= 1;
	end
	else begin
		rShoot <= 0;
	end
end
assign oPower = rShoot;
endmodule