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
output	reg		oPower;

reg		[15:0]	rCount_1ms;
reg 	[15:0]	rCount_3s;	//wait
reg				rClk_1ms;
reg		[7:0]	rShoot_power;
reg		[7:0]	rPower;
reg				rShoot_EN;


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
		rCount_3s 	 <= 0;
		rShoot_EN	 <= 0;
		rPower		 <= 0;
		rShoot_power <= 0;
	end
	else if(rShoot_EN && rPower)begin
		if(rShoot_power < rPower)begin
			rShoot_power <= rShoot_power+1;
			oPower <= 1;
		end
		else begin
			oPower <= 0;
			rPower <= 0;
			rShoot_power <= 0;
			rShoot_EN <= 0;			
		end
	end
	else begin
		rPower <= iPower;
		if(rCount_3s < 3000) begin
			rCount_3s <= rCount_3s+1;
		end
		else begin
			rShoot_EN <= 1;
			rCount_3s <= 0;
		end
	end
end
endmodule