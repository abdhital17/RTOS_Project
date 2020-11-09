; MPU assembly functions
; used in the MPU code

	.def unprivilegedMode
	.def privilegedMode
	.def setPSPaddress
	.def getPSPaddress
	.def getMSPaddress
	.def setASPbit
	.def getSVCNumber


.thumb
.const

.text

setPSPaddress:
		MSR 	PSP, R0
		BX		LR

getPSPaddress:
		MRS 	R0, PSP
		BX 		LR

getMSPaddress:
		MRS 	R0, MSP
		BX		LR

unprivilegedMode:
			MRS 	R4, CONTROL
			ORR 	R4, R4, #0x01
			MSR		CONTROL, R4
			BX 		LR

privilegedMode:
			MRS 	R4, CONTROL
			ORR 	R4, R4, #0x00
			MSR		CONTROL, R4
			BX 		LR

setASPbit:
			MRS 	R4, CONTROL
			ORR 	R4, R4, #0x2
			MSR		CONTROL, R4
			BX 		LR

getSVCNumber:
			MRS		R0, PSP
			LDR		R0, [R0, #24]
			LDR		R0, [R0, #-2]
			;SUB		R0, R0, #4;
			AND		R0, R0, #0xFF
			BX		LR


.endm


