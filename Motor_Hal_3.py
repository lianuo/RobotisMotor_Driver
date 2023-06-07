import os,platform
from dynamixel_sdk import * 
import numpy as np
import struct

BAUDRATE                    = 1000000

platform_now = platform.platform().lower()

sport="COM5"
if "linux" in platform_now:
    sport='/dev/ttyUSB0'

dict_addtable={}
dict_ax12={}
dict_ax12['Torque Enable']=24
dict_ax12['LED']=25
dict_ax12['Goal Position']=30
dict_ax12['Moving Speed']=32
dict_ax12['Torque Limit']=34
dict_ax12['Present Position']=36
dict_ax12['Present Speed']=38
dict_ax12['Present Load']=40
dict_ax12['Present Voltage']=42
dict_ax12['Present Temperature']=43

dict_ax12['zero']=512#rpm
dict_ax12['range_pose']=[0,1023]
dict_ax12['range_spd']=[0,2047]
dict_ax12['unit_pose']=0.29
dict_ax12['unit_spd']=0.111#rpm


dict_addtable['ax12']=dict_ax12

dict_mx28={}

dict_mx28['Torque Enable']=64
dict_mx28['LED']=65
dict_mx28['D Gain']=80
dict_mx28['I Gain']=82
dict_mx28['P Gain']=84
dict_mx28['Moving Speed'] =112#104
dict_mx28['Goal Position']=116
#dict_mx28['Torque Limit']=34
dict_mx28['Present Load']=126
dict_mx28['Present Speed']=128
dict_mx28['Present Position']=132


dict_mx28['Present Voltage']=144
dict_mx28['Present Temperature']=146
dict_mx28['Current']=126

dict_mx28['zero']=2048#rpm
dict_mx28['range_pose']=[0,4095]
dict_mx28['unit_pose']=0.088
dict_mx28['range_spd']=[0,2047]
dict_mx28['unit_spd']=0.229#0.114#rpm

dict_addtable['mx28']=dict_mx28



dict_mx64={}

dict_mx64['Torque Enable']=64
dict_mx64['LED']=65
dict_mx64['D Gain']=80
dict_mx64['I Gain']=82
dict_mx64['P Gain']=84
dict_mx64['Goal Position']=116
dict_mx64['Moving Speed']=112#104
#dict_mx64['Torque Limit']=34
dict_mx64['Present Position']=132
dict_mx64['Present Speed']=128
dict_mx64['Present Load']=126
dict_mx64['Present Voltage']=144
dict_mx64['Present Temperature']=146
dict_mx64['Current']=126

dict_mx64['zero']=2048#rpm
dict_mx64['range_pose']=[0,4095]
dict_mx64['unit_pose']=0.088
dict_mx64['range_spd']=[0,2047]
dict_mx64['unit_spd']=0.229#0.114#rpm

dict_addtable['mx64']=dict_mx64

dict_mx106={}

dict_mx106['Torque Enable']=64
dict_mx106['LED']=65
dict_mx106['D Gain']=80
dict_mx106['I Gain']=82
dict_mx106['P Gain']=84
dict_mx106['Goal Position']=116
dict_mx106['Moving Speed']=112#104
#dict_mx106['Torque Limit']=34
dict_mx106['Present Position']=132
dict_mx106['Present Speed']=128
dict_mx106['Present Load']=126
dict_mx106['Present Voltage']=144
dict_mx106['Present Temperature']=146
dict_mx106['Current']=126

dict_mx106['zero']=2048#rpm
dict_mx106['range_pose']=[0,4095]
dict_mx106['unit_pose']=0.088
dict_mx106['range_spd']=[0,2047]
dict_mx106['unit_spd']=0.229#0.114#rpm


dict_addtable['mx106']=dict_mx106



portHandler = PortHandler(sport)
packetHandler1 = PacketHandler(1.0)
packetHandler2 = PacketHandler(2.0)
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    quit()

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
#DXL_MINIMUM_POSITION_VALUE  = 400           # Dynamixel will rotate between this value
#DXL_MAXIMUM_POSITION_VALUE  = 600            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

list_prt1=['ax12']
list_prt2=['mx28','mx64','mx106']
class Motor:
    def __init__(self,name,mtype,mid,mmin,mmax,minit,dir,offset) -> None:       
        self.name=name
        self.type=mtype#,'ax12'#
        self.prot=2
        if self.type in list_prt1:
            self.prot=1
        self.id=mid    

        self.spd_init=0
        self.spd_goal=0
        self.spd_now=0

        self.pos_min=mmin
        self.pos_max=mmax
        self.pos_init=minit
        self.pos_goal_degree=0
        self.pos_now_degree=0
        self.pos_offset=offset

        self.load_now=0

        self.dir=dir#revers -1
        self.table=dict_addtable[self.type]
        #init torque
        if self.type in list_prt2:
            self.packetHandler=packetHandler2
        else:
            self.packetHandler=packetHandler1

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, self.id, self.table['Torque Enable'], TORQUE_ENABLE)
        self.check_err(dxl_comm_result, dxl_error)

        self.sig=0#Semaphore

        self.Syc()
    def FromDegree2Byte(self,degree):
        d_byte=(int)((degree+self.pos_offset)*self.dir/self.table['unit_pose']+self.table['zero'])
        return d_byte
    def FromByte2Degree(self,d_byte):
        degree=(d_byte-self.table['zero'])*self.table['unit_pose']*self.dir-self.pos_offset
        return degree
    def FromDegree2Rad(self,d_degree):
        rad=d_degree*3.1415/180
        return rad
     
    def ReadState(self):
        if self.sig!=0:
            return False
        self.sig=1
        data, result, error=-9000,-9000,-9000
        if self.prot==1:
            data, result, error = self.packetHandler.readTxRx(portHandler, self.id, self.table['Present Position'],6)
        else:#prot==2:
            data, result, error = self.packetHandler.readTxRx(portHandler, self.id, self.table['Present Load'],10)

        if result!=COMM_SUCCESS:
            self.check_err(result,error,str="--ReadState--")
            data_read = 0,0,0
        else:
            #pos=DXL_MAKEWORD(data[0], data[1])
            data_read = data,0,0
        if result==COMM_SUCCESS:
            if self.prot==1:
                pos=DXL_MAKEWORD(data[0], data[1])
                speed_16=DXL_MAKEWORD(data[2], data[3])
                load_16=DXL_MAKEWORD(data[4], data[5])

                #speed=struct.unpack("<h",speed_16)
                #load=struct.unpack("<h",load_16)
                data_read = pos,speed_16,load_16
            else:
                pos=DXL_MAKEWORD(data[6], data[7])
                speed_16=DXL_MAKEWORD(data[2], data[3])
                load_16=DXL_MAKEWORD(data[0], data[1])
                
                speed=np.int16(speed_16)
                load=np.int16(load_16)
                data_read = pos,speed,load
        else:
            data_read=self.table['zero'],0,0
        self.sig=0
        return data_read, result, error
        #dxl_present_position,dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.id, self.table['Present Position'])
    def Move(self,pos_degree,time):#l1, pos -180~180degree ,time s
        self.Syc()
        self.pos_goal_degree=pos_degree        
        pos_b=self.FromDegree2Byte(pos_degree)        
        dd=abs(self.pos_now_degree-pos_degree)*1.0      
        self.spd_goal=dd/time#deregge/sencond
        spd_b=(int)(self.spd_goal*60/360/self.table['unit_spd'])
        res=self.Move_l0(pos_b,spd_b)     
    def Move_l0(self,pos_b,spd_b):#l0, 512 pos ,and spd byte    
        if self.sig!=0:
            return False
        self.sig=1
        if self.prot==2:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler,  self.id, self.table['Moving Speed'],spd_b)
            self.check_err(dxl_comm_result, dxl_error,str="--Move_l0--")
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler,  self.id, self.table['Goal Position'],pos_b)
            self.check_err(dxl_comm_result, dxl_error,str="--Move_l0--")
        else:
            # dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(portHandler,  self.id, self.table['Moving Speed'],spd_b)
            # self.check_err(dxl_comm_result, dxl_error,str="--Move_l0--")
            # dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(portHandler,  self.id, self.table['Goal Position'],pos_b)
            # self.check_err(dxl_comm_result, dxl_error,str="--Move_l0--")
        #  
            data_write = [DXL_LOBYTE(pos_b),
                        DXL_HIBYTE(pos_b),
                        DXL_LOBYTE(spd_b),
                        DXL_HIBYTE(spd_b)]
            res=self.packetHandler.writeTxRx(portHandler, self.id, self.table['Goal Position'], 4, data_write)
        self.sig=0
        return  True
    #     #return time#estimate
    # def GetPos(self):
    #     dxl_present_position,dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.id, self.table['Present Position'])
    #     check_err(dxl_comm_result, dxl_error)
    #     return dxl_present_position
    def Syc(self):

        #self.pos_goal_degree----------------------
        #self.pos_now=self.GetPos()
        self.Torque_Enable()
        data_read, result, error=self.ReadState()
        num=self.check_err(result, error,str="---Syc---")
        if num!=0:
            #--
            if error==128:
                print("  id %d overload--,reboot..\n"%(self.id))
                
                self.Reboot()
                # self.Torque_Enable()
                self.Move(self.pos_goal_degree,5)
            
        # if error
        pos_b,speed_b,load_b=data_read
        self.pos_now_degree=self.FromByte2Degree(pos_b)
        self.pos_now_rad=self.FromDegree2Rad(self.pos_now_degree)
        self.spd_now=speed_b
        self.load_now=load_b
        # if result!=COMM_SUCCESS:
        #     print("----Syc---err--\n")
        return self.pos_now_rad,self.spd_now,self.load_now
    def Torque_Enable(self):
        if self.sig!=0:
            return False
        self.sig=1        
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, self.id, self.table['Torque Enable'], TORQUE_ENABLE)
        self.sig=0
        self.check_err(dxl_comm_result, dxl_error,str="--Torque_Enable--")
    def Torque_Disable(self):
        if self.sig!=0:
            return False
        self.sig=1   
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, self.id, self.table['Torque Enable'], TORQUE_DISABLE)
        self.sig=0   
        self.check_err(dxl_comm_result, dxl_error)
    def check_err(self,dxl_comm_result, dxl_error,str=""):
        errs=""
        #print(str+"\t")
        errs+=str+"\t"
        num=0
        if dxl_comm_result != COMM_SUCCESS:
            #print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            errs+="%s" % self.packetHandler.getTxRxResult(dxl_comm_result)
            num+=1
        elif dxl_error != 0:
            #print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            errs+="%s" % self.packetHandler.getRxPacketError(dxl_error)
            num+=1
        if num>0:
            print(errs+"\n")
        return num
    def Reboot(self):
        self.Torque_Disable()
        dxl_comm_result, dxl_error = self.packetHandler.reboot(portHandler, self.id)
        self.check_err(dxl_comm_result, dxl_error,str="--Reboot--")
        time.sleep(1)
        self.Torque_Enable()
def CheckSpeed():
    j17=Motor('j17','ax12',17,-45,45,0,1,0)
    j15=Motor('j15','mx28',15,-45,45,0,1,0)
    while 1:
        j15.Move(20,3)
        j17.Move(20,3)
        time.sleep(3)
        pos_now_rad,spd_now,load_now=j15.Syc()
        print("p %f \t s %d \t load %d \n"%(pos_now_rad*180/3.14,spd_now,load_now))
        pos_now_rad,spd_now,load_now=j17.Syc()
        print("17  p %f \t s %d \t load %d \n"%(pos_now_rad*180/3.14,spd_now,load_now))
        #j15.Reboot()
        j15.Move(-20,3)
        j17.Move(-20,3)
        time.sleep(3)
        pos_now_rad,spd_now,load_now=j15.Syc()
        print("p %f \t s %d \t load %d \n"%(pos_now_rad*180/3.14,spd_now,load_now))
        pos_now_rad,spd_now,load_now=j17.Syc()
        print("17  p %f \t s %d \t load %d \n"%(pos_now_rad*180/3.14,spd_now,load_now))
        #j15.Reboot()
def CheckOverload():
    j11=Motor('r_shoulder_roll','mx64', 11, -10,    10,     0,    -1,90)
    dt=3
    while 1:
        j11.Move(0,dt)
        time.sleep(dt)
        
        #j15.Reboot()
        j11.Move(-90,dt)
        time.sleep(10)        

if __name__=="__main__":
    CheckSpeed()