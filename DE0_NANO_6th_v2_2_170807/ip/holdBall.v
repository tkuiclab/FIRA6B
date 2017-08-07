module holdBall(iC,oL,oR,iCMD1,iCMD2/*,oL1,oR1*/);
inout iC;
input [7:0]iCMD1,iCMD2;
output reg oL,oR/*,oL1,oR1*/;
reg [1:0]cont;
reg [32:0]con;
reg [6:0]spd1,spd2;
always@(posedge iC)begin

	spd1 <= iCMD1[6:0];
	spd2 <= iCMD2[6:0];
	
	if(spd1 == 0 && spd2 == 0)begin
		oL = 1'b1;
		oR = 1'b1;
	end else begin
		if(iCMD1[7] == 0 && iCMD2[7] == 1)begin
			oL <= 1'b1;
			oR <= 1'b1;
		end else begin 
			oL <= 1'b0;
			oR <= 1'b0;
		end
	end
	/*if(iD1 == 1 || iD2 == 1)begin
		oL <= 1'b1;
		oR <= 1'b1;
		oL1 <= 1'b1;
		oR1 <= 1'b1;
	end else	begin
		oL <= 1'b0;
		oR <= 1'b0;
		oL1 <= 1'b0;
		oR1 <= 1'b0;
	end*/
	/*if(con == 50000000)begin
		con <= 0;
		cont <= cont + 1;
	end else begin
		con <= con + 1;
		
	end
	
	if(cont == 1 | cont == 0)begin
		oL = 1;
		oR = 0;
		oL1 = 1;
		oR1 = 0;
	end else begin
		oL = 0;
		oR = 1;
		oL1 = 0;
		oR1 = 1;
	end*/
end
endmodule 