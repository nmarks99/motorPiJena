drvAsynIPPortConfigure("MOXA1", "10.54.115.118:4002")
asynOctetSetInputEos("MOXA1",0,"\r\n")
asynOctetSetOutputEos("MOXA1",0,"\r\n")

# PiJenaMotorCreateController(port name, asyn port, num axes, moving poll period ms, idle poll period ms)
PiJenaMotorCreateController("JENA1", "MOXA1", 3, 100, 1000)
dbLoadTemplate("PiJenaAsyn.substitutions", "P=piJena:")

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=piJena:, R=asyn_1, PORT=MOXA1, ADDR=0, OMAX=$(OUT_BUFF=1000), IMAX=$(IN_BUFF=1000)")
