// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert serial to command
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/07/19 :|  Initial Version
// --------------------------------------------------------------------

module Serial2CMD (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 		// 50MHz
input				iRst_n,		// Reset
input				iRx_ready,
input		[7:0]	iData,		// Data
output	reg	[7:0]	oCMD_Motor1,	// Command of motor1
output	reg	[7:0]	oCMD_Motor2,	// Command of motor2
output	reg	[7:0]	oCMD_Motor3,	// Command of motor3
output	reg	[7:0]	oCMD_Motor4,	// Command of motor4
output	reg	[7:0]	oAX_12,
output	reg			okick,			// shoot a ball at the goal
output	reg			oBrush,
output	reg			oRx_done
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	8;
// differentiate state in order to change state
parameter DATA0	=	8'b00000001;
parameter DATA1	=	8'b00000010;
parameter DATA2	=	8'b00000100;
parameter DATA3	=	8'b00001000;
parameter DATA4	=	8'b00010000;
parameter DATA5	=	8'b00100000;
parameter DATA6	=	8'b01000000;
parameter END	=	8'b10000000;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
//	divide information to 6 part and 8 bits per part
reg		[7:0]	rData_0, rData_1, rData_2, rData_3, rData_4, rData_5,rData_6;
reg		[7:0]	rCheck_sum;
reg	[SIZE-1:0]	state;

reg				rRx_ready;


//=============================================================================
// Structural coding
//=============================================================================

always @(posedge iCLK) begin
	if(!iRst_n) begin		// Reset
		oRx_done	<=	0;
		rData_0		<=	0;
		rData_1		<=	0;
		rData_2		<=	0;
		rData_3		<=	0;
		rData_4		<=	0;
		rData_5		<=	0;
		rData_6		<=	0;
		state		<=	DATA0;
	end
	// Take apart Data
	else if(~rRx_ready & iRx_ready) begin
		case(state)
			DATA0:	
				if( iData == 8'hFF ) begin	// when getting initiation packet, state jump next
					rData_0	<=	iData;
					state	<=	DATA1;
				end
			DATA1:
				if( iData == 8'hFA ) begin	//when getting second initiation packet, start to receive and transmit Data
					rData_1	<=	iData;
					state	<=	DATA2;
				end
				else begin
					state	<=	DATA0;
				end
			DATA2:
				begin
					rData_2	<=	iData;
					state	<=	DATA3;
				end
			DATA3:
				begin
					rData_3	<=	iData;
					state	<=	DATA4;
				end
			DATA4:
				begin
					rData_4	<=	iData;
					state	<=	DATA5;
				end
			DATA5:
				begin
					rData_5	<=	iData;
					state	<=	DATA6;
				end
			DATA6:
				begin
					rData_6	<=	iData;
					oRx_done	<=	1;
					state	<=	DATA0;
				end
		endcase
	end
	else begin
//		oCMD_Motor1	<=	rData_2;		// give oCMD_Motor1 Data
//		oCMD_Motor2	<=	rData_3;		// give oCMD_Motor2 Data
//		oCMD_Motor3	<=	rData_4;		// give oCMD_Motor3 Data
//		oCMD_Motor4	<=	rData_5;		// give oCMD_Motor4 Data
//		okick		<=	rData_6[7];		// shoot a ball at the goal
		rCheck_sum <= (rData_2+rData_3)+(rData_4+rData_5);
		if((rCheck_sum==rData_6)&&(rData_0==8'hFF)&&(rData_1==8'hFA))begin
			oCMD_Motor1	<=	rData_2;		// give oCMD_Motor1 Data
			oCMD_Motor2	<=	rData_3;		// give oCMD_Motor2 Data
			oCMD_Motor3	<=	rData_4;		// give oCMD_Motor3 Data
			oCMD_Motor4	<=	rData_5;		// give oCMD_Motor4 Data
//			okick		<=	rData_6[7];		// shoot a ball at the goal
		end
		else begin
			oCMD_Motor1	<=	oCMD_Motor1;		// give oCMD_Motor1 Data
			oCMD_Motor2	<=	oCMD_Motor1;		// give oCMD_Motor2 Data
			oCMD_Motor3	<=	oCMD_Motor1;		// give oCMD_Motor3 Data
			oCMD_Motor4	<=	oCMD_Motor1;		// give oCMD_Motor4 Data
		end
		oRx_done	<=	0;
	end
	rRx_ready	<=	iRx_ready;
end

endmodule
