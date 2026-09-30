$ErrorActionPreference = 'Stop'
$protocol = Get-Content "$PSScriptRoot/../main/usb_protocol.h" -Raw
$serial = Get-Content "$PSScriptRoot/../main/serial_ctrl.c" -Raw
$parser = $protocol.Substring($protocol.IndexOf('static inline bool usb_parser_push'))
$logic = $serial.Substring($serial.IndexOf('serial_ctrl_stats_t serial_ctrl_get_stats'), $serial.IndexOf('static void rx_task') - $serial.IndexOf('serial_ctrl_stats_t serial_ctrl_get_stats'))
$body = ($parser + $logic).Replace('uint8_t byte', 'uint8_t octet').Replace('= byte;', '= octet;')
$body = $body -replace 'portENTER_CRITICAL\([^;]*;', '' -replace 'portEXIT_CRITICAL\([^;]*;', ''
$body = $body.Replace('static inline ', 'static ').Replace('const usb_frame_t *frame', 'usb_frame_t frame').Replace('usb_parser_t *p', 'usb_parser_t p').Replace('usb_frame_t *frame', 'usb_frame_t frame').Replace('uint32_t *errors', 'ref uint errors')
$body = $body.Replace('p->', 'p.').Replace('frame->', 'frame.').Replace('(*errors)++', 'errors++').Replace('*frame', 'frame')
$body = $body.Replace('usb_u16(p.bytes + 14)', 'usb_u16(p.bytes, 14)').Replace('usb_u32(p.bytes + 4)', 'usb_u32(p.bytes, 4)').Replace('usb_u16(p.bytes + 8 + i * 2)', 'usb_u16(p.bytes, 8 + i * 2)')
$body = $body.Replace('frame.value[0] & 1, frame.value[0] & 2, frame.value[1]', '(frame.value[0] & 1) != 0, (frame.value[0] & 2) != 0, (uint)frame.value[1]')
$body = $body -replace '\buint32_t\b','uint' -replace '\buint16_t\b','ushort' -replace '\buint8_t\b','byte' -replace '\bint16_t\b','short' -replace '\bint64_t\b','long' -replace '\bsize_t\b','int' -replace '\bunsigned\b','uint' -replace '\(void\)','()'
$body = $body.Replace('serial_ctrl_stats_t serial_ctrl_get_stats', 'static serial_ctrl_stats_t serial_ctrl_get_stats').Replace('bool serial_ctrl_owns_motion','static bool serial_ctrl_owns_motion').Replace('esp_err_t serial_ctrl_configure','static int serial_ctrl_configure').Replace('void serial_ctrl_poll','static void serial_ctrl_poll').Replace('usb_frame_t command;', 'usb_frame_t command = new usb_frame_t();')
$harness = @"
using System;
public static class UsbTests {
const uint USB_MOTION_MAGIC=0x4D53494D, USB_CONFIG_MAGIC=0x4643534D;
const int USB_FRAME_SIZE=16, ESP_OK=0, ESP_ERR_INVALID_ARG=1, ESP_ERR_INVALID_STATE=2, PID_MODE_FOLLOW=0;
class usb_parser_t {public byte[] bytes=new byte[16];public int used;}
class usb_frame_t {public uint magic,sequence;public short[] value=new short[3];}
struct serial_ctrl_stats_t {public bool ready,control_enabled,telemetry_enabled,timed_out;public ushort telemetry_hz;public uint rejected,received,coalesced;}
static serial_ctrl_stats_t s_stats;
static usb_frame_t s_latest;
static bool s_pending,s_have_sequence,s_stop_pending;
static uint s_sequence;
static long s_last_rx_us,clock;
static bool armed,estop,tuning;
static float[] targets=new float[3];
static long esp_timer_get_time(){return clock;}
static bool motor_get_estop(){return estop;}
static bool pid_is_autotuning(){return tuning;}
static void motor_arm(bool value){armed=value;}
static void pid_reset_integrals(){}
static void pid_set_mode(int mode){}
static void pid_set_target_angle(int joint,float angle){targets[joint-1]=angle;armed=true;}
static ushort usb_u16(byte[] b,int offset=0){return (ushort)(b[offset]|b[offset+1]<<8);}
static uint usb_u32(byte[] b,int offset=0){return (uint)(usb_u16(b,offset)|((uint)usb_u16(b,offset+2)<<16));}
$body
static void Check(bool value,string message){if(!value)throw new Exception(message);}
static byte[] Packet(uint magic,uint seq,short a,short b,short c){
 byte[] p=new byte[16];Array.Copy(BitConverter.GetBytes(magic),0,p,0,4);Array.Copy(BitConverter.GetBytes(seq),0,p,4,4);
 Array.Copy(BitConverter.GetBytes(a),0,p,8,2);Array.Copy(BitConverter.GetBytes(b),0,p,10,2);Array.Copy(BitConverter.GetBytes(c),0,p,12,2);
 ushort sum=0;for(int i=0;i<14;i++)sum+=p[i];Array.Copy(BitConverter.GetBytes(sum),0,p,14,2);return p;
}
static usb_frame_t Motion(uint seq,short a){return new usb_frame_t{magic=USB_MOTION_MAGIC,sequence=seq,value=new short[]{a,1800,1800}};}
public static void Run(){
 var parser=new usb_parser_t();var frame=new usb_frame_t();uint errors=0;int frames=0;
 byte[] good=Packet(USB_MOTION_MAGIC,0xFFFFFFFF,1000,1800,3595);
 byte[] bad=(byte[])good.Clone();bad[8]^=1;
 foreach(byte b in new byte[]{0,99,23,77,73}) Check(!usb_parser_push(parser,b,frame,ref errors),"Noise accepted");
 foreach(byte b in bad)if(usb_parser_push(parser,b,frame,ref errors))frames++;
 foreach(byte b in good)if(usb_parser_push(parser,b,frame,ref errors))frames++;
 Check(frames==1 && errors==1 && frame.value[0]==1000 && frame.sequence==0xFFFFFFFF,"Corrupt packet recovery failed");
 for(int repeat=0;repeat<1000;repeat++)foreach(byte b in good)if(usb_parser_push(parser,b,frame,ref errors))frames++;
 Check(frames==1001,"Back-to-back packets lost");
 Check(usb_sequence_newer(0,0xFFFFFFFF) && !usb_sequence_newer(2,2) && !usb_sequence_newer(1,2),"Sequence wrap/replay failed");
 s_stats=new serial_ctrl_stats_t{ready=true,telemetry_hz=100};clock=1000;estop=false;tuning=false;
 Check(serial_ctrl_configure(false,true,100)==ESP_OK,"Telemetry enable failed");receive_frame(Motion(0,1000));serial_ctrl_poll();Check(!armed,"Telemetry-only mode moved motor");
 Check(serial_ctrl_configure(true,true,100)==ESP_OK,"Control enable failed");serial_ctrl_poll();Check(!armed,"Configuration alone moved motor");
 receive_frame(Motion(0xFFFFFFFF,1000));receive_frame(Motion(0,1200));serial_ctrl_poll();Check(armed && targets[0]==120 && s_stats.coalesced==1,"Latest mailbox or sequence wrap failed");
 receive_frame(Motion(0,2000));receive_frame(Motion(0xFFFFFFFF,2000));serial_ctrl_poll();Check(targets[0]==120,"Replay applied");
 clock+=100001;serial_ctrl_poll();Check(!armed && s_stats.timed_out,"Timeout did not disarm");receive_frame(Motion(1,2000));serial_ctrl_poll();Check(!armed,"Late frame bypassed timeout latch");
 serial_ctrl_configure(true,true,100);receive_frame(Motion(0,900));serial_ctrl_poll();Check(armed && targets[0]==90,"Explicit session restart failed");
 estop=true;serial_ctrl_poll();estop=false;receive_frame(Motion(1,1200));serial_ctrl_poll();Check(!armed && s_stats.timed_out,"E-stop restart latch failed");
 tuning=true;Check(serial_ctrl_configure(true,true,100)!=ESP_OK,"Calibration was interrupted");
 Check(serial_ctrl_configure(false,true,101)!=ESP_OK,"Invalid telemetry rate accepted");
 Console.WriteLine("PASS production USB parser, checksum recovery, batching, sequence wrap, source gating, timeout and E-stop latches");
}
}
"@
# Protocol and mailbox logic are extracted from firmware; adapters supply SDK calls.
# C integer narrowing has explicit equivalents below.
$harness=$harness.Replace('s_stats.telemetry_hz = hz;', 's_stats.telemetry_hz = (ushort)hz;')
Add-Type -TypeDefinition $harness
[UsbTests]::Run()
