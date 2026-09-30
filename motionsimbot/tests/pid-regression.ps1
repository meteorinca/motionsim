$ErrorActionPreference = 'Stop'
$source = Get-Content "$PSScriptRoot/../main/pid_controller.c" -Raw
function Section([string]$start, [string]$end) {
    $a = $source.IndexOf($start)
    $b = $source.IndexOf($end, $a + $start.Length)
    if ($a -lt 0 -or $b -lt 0) { throw 'Source section missing' }
    return $source.Substring($a, $b - $a)
}
$profile = Section '// 1. Read all sensors' '// 2. Multi-Hazard'
$output = Section '// 7. Closed-Loop' '// Store last actuals'
$setter = Section 'void pid_set_target_angle(' 'float pid_get_target_angle('
$mode = Section 'void pid_set_mode(' 'pid_control_mode_t pid_get_mode('
$helpers = Section 'static float shortest_angle_diff(' 'static void validate_new_params('
$tuner = Section 'static void tune_hold_current(' '// -------------------------------------------------------------'
$reset = Section 'void pid_reset_integrals(' '// Multi-point tuning'
$body = $reset + $tuner + $helpers + $setter + $mode + "`nstatic void Tick() { const float dt = 0.01f;`n" + $profile + $output + @'
for(int i=0;i<3;i++) s_last_actuals[i]=s_actuals[i];
}
'@
$body = $body -replace 'int16_t', 'short' -replace 'isfinite\(', 'Finite('
$body = $body -replace 'ESP_LOGI\([^;]*;', '' -replace 'snprintf\([\s\S]*?\);', '' -replace 'const char \*reason', 'string reason' -replace '\bunsigned\b', 'uint' -replace '\buint32_t\b', 'uint' -replace '\(void\)', '()'
$body = $body -replace 'void pid_set_target_angle', 'static void pid_set_target_angle'
$body = $body -replace 's_terms\[i\] = \(control_terms_t\)\{[^;]*;', ''
$body = $body -replace 'void pid_reset_integrals\(\) \{', 'static void pid_reset_integrals() {resets++;'
$body = $body -replace 'void pid_set_mode', 'static void pid_set_mode'
$prefix = @'
using System;
public static class PidRegression {
const int JOINT_COUNT=3, MOTOR_CLAMP_DUTY=819;
struct Params { public float kp,ki,kd,kff,integral_max,max_speed_deg_s,max_accel_deg_s2,min_duty,deadband_deg,max_duty_limit,soft_min_deg,soft_max_deg,beta,k_aw,breakaway_fwd,breakaway_rev,coulomb_fwd,coulomb_rev; }
static Params s_params; static bool s_trace_frozen=false;
const pid_control_mode_t PID_MODE_FOLLOW=pid_control_mode_t.FOLLOW;
enum TuneState { AUTOTUNE_BASELINE, AUTOTUNE_STEP_FWD, AUTOTUNE_FAILED, AUTOTUNE_SUCCESS }
const TuneState AUTOTUNE_BASELINE=TuneState.AUTOTUNE_BASELINE,AUTOTUNE_STEP_FWD=TuneState.AUTOTUNE_STEP_FWD,AUTOTUNE_FAILED=TuneState.AUTOTUNE_FAILED,AUTOTUNE_SUCCESS=TuneState.AUTOTUNE_SUCCESS;
static TuneState s_autotune_state;
static Params s_autotune_original;
static int s_autotune_joint=1;
static uint s_tune_leg,s_autotune_start_ms,s_tune_settled_ms,s_tune_samples,s_tune_baseline_samples,s_tune_residual_samples,s_params_revision;
static bool s_tune_validation;
static float s_tune_goal,s_tune_leg_start,s_autotune_start_angle,s_tune_iae,s_tune_baseline_iae,s_tune_residual,s_tune_overshoot;
static float[] s_tune_offsets={5,0,-5,0,10,0,-10,0,15,0,-15,0};
const uint TUNE_LEGS=12;
static void motor_stop_all() { }
static void TuneTest(bool stationary,bool reject) {
 Reset(90);s_autotune_original=s_params;s_autotune_start_angle=90;s_autotune_state=AUTOTUNE_BASELINE;
 s_autotune_start_ms=0;s_tune_settled_ms=0;s_tune_leg=0;s_tune_validation=false;
 s_tune_iae=0;s_tune_samples=0;s_tune_residual=0;s_tune_residual_samples=0;s_tune_overshoot=0;
 s_control_mode=pid_control_mode_t.AUTOTUNE;
 uint finish=0;
 for(uint now=10;now<160000;now+=10) {
  if(s_autotune_state==AUTOTUNE_STEP_FWD && !stationary) {
   s_unwrapped_actuals[0]=s_tune_goal+(reject && s_tune_validation ? .2f:0);s_actuals[0]=wrap_angle(s_unwrapped_actuals[0]);
  }
  s_d_filtered[0]=0;process_autotune(now);
  if(s_autotune_state==AUTOTUNE_SUCCESS || s_autotune_state==AUTOTUNE_FAILED) {finish=now;break;}
 }
 Check(finish>0,"Tuner never finished");
 if(stationary || reject) { Check(s_trace_frozen,"Failure did not freeze diagnostics");
  Check(s_autotune_state==AUTOTUNE_FAILED,"Bad calibration accepted");
  Check(s_params.kp==s_autotune_original.kp && s_params.ki==s_autotune_original.ki && s_params.kd==s_autotune_original.kd,"Failed gains not restored");
 } else {
  Check(s_autotune_state==AUTOTUNE_SUCCESS && finish>=49000,"Calibration skipped measurement/validation moves");
  Check(s_params.kp!=s_autotune_original.kp,"Validated gains not applied");
 }
 Console.WriteLine("PASS tuner stationary="+stationary+" reject="+reject+" duration="+finish);
}
enum pid_control_mode_t { FOLLOW, MANUAL, AUTOTUNE }; static pid_control_mode_t s_control_mode=pid_control_mode_t.FOLLOW; static int resets=0; static int[] s_runaway_counters=new int[3],s_stall_counters=new int[3],s_magnet_err_counters=new int[3];
static float[] s_targets=new float[3],s_actuals=new float[3],s_unwrapped_actuals=new float[3],s_unwrapped_targets=new float[3],s_unwrapped_profiled=new float[3],s_mapped_targets=new float[3],s_profiled_vels=new float[3],s_last_actuals=new float[3],s_stall_ref_angles=new float[3],s_d_filtered=new float[3],s_errors=new float[3],s_integrals=new float[3],s_breakaway_elapsed=new float[3];
static bool[] s_target_initialized=new bool[3],s_angle_initialized=new bool[3],s_target_mapped=new bool[3];
static int[] s_friction_direction=new int[3];
static short[] s_last_cmd_duty=new short[3],duties=new short[3];
static float sensor; static float[] s_move_starts=new float[3]; static bool tuning=false; static bool pid_is_autotuning() { return tuning; }
static bool Finite(float x) { return !float.IsNaN(x) && !float.IsInfinity(x); }
static float fabsf(float x) { return Math.Abs(x); }
static float fminf(float a,float b) { return Math.Min(a,b); }
static float fmaxf(float a,float b) { return Math.Max(a,b); }
static float fmodf(float a,float b) { return a%b; }
static bool as5600_is_detected(int i) { return i==0; }
static float hall_read_angle(int i) { return sensor; }
static bool motor_get_estop() { return false; }
static bool motor_is_armed() { return true; }
static void motor_arm(bool x) { }
static bool motor_is_jogging(int i) { return false; }
static void motor_set_duty(int i,short duty) { duties[i-1]=duty; }
static void Check(bool value,string message) { if(!value) throw new Exception(message); }
static void Reset(float angle) {
 s_params=new Params {kp=14,ki=.2f,kd=1.8f,kff=2.2f,integral_max=200,max_speed_deg_s=80,max_accel_deg_s2=180,min_duty=90,deadband_deg=.25f,max_duty_limit=700,soft_min_deg=0,soft_max_deg=360,beta=.6f,k_aw=.5f};
 s_target_initialized[0]=false;s_angle_initialized[0]=false;s_target_mapped[0]=false;s_integrals[0]=0;s_d_filtered[0]=0;s_last_cmd_duty[0]=0;s_breakaway_elapsed[0]=0;s_friction_direction[0]=0;sensor=angle;Tick();
}
static void Move(float start,float target,float expected) {
 Reset(start);pid_set_target_angle(1,target);
 for(int n=0;n<2000;n++) {Tick();sensor=wrap_angle(s_unwrapped_profiled[0]);}
 Check(Math.Abs(s_unwrapped_targets[0]-expected)<.01f,"Wrong destination "+start+" -> "+target);
 Check(Math.Abs(s_unwrapped_profiled[0]-expected)<.11f,"Profile failed to settle");
 Check(duties[0]==0,"Motor did not settle");
 Console.WriteLine("PASS move "+start+" -> "+target+" (continuous "+expected+")");
}
static void PlantMove(float start,float target,float gain,float drag) {
 Reset(start);float position=start,velocity=0;pid_set_target_angle(1,target);
 for(int n=0;n<4000;n++) {
  Tick();
  float effort=Math.Sign(duties[0])*Math.Max(0,Math.Abs(duties[0])-75);
  velocity+=(gain*effort-drag*velocity)*.01f;
  position+=velocity*.01f;sensor=wrap_angle(position);
  Check(Math.Abs(position-start)<220,"Plant made a full revolution");
 }
 Check(Math.Abs(shortest_angle_diff(target,sensor))<1,"Plant did not settle: "+sensor);
 Console.WriteLine("PASS simulated motor "+start+" -> "+target+" gain="+gain+" drag="+drag);
}
static void TunePlantTest() {
 Reset(90);s_autotune_original=s_params;s_autotune_start_angle=90;s_autotune_state=AUTOTUNE_BASELINE;
 s_autotune_start_ms=0;s_tune_settled_ms=0;s_tune_leg=0;s_tune_validation=false;
 s_tune_iae=0;s_tune_samples=0;s_tune_residual=0;s_tune_residual_samples=0;s_tune_overshoot=0;
 s_control_mode=pid_control_mode_t.AUTOTUNE;tuning=true;
 float position=90,velocity=0,min=90,max=90;
 uint finish=0;
 for(uint now=10;now<160000;now+=10) {
  Tick();process_autotune(now);
  float effort=Math.Sign(duties[0])*Math.Max(0,Math.Abs(duties[0])-75);
  velocity+=(2*effort-4*velocity)*.01f;
  position+=velocity*.01f;sensor=wrap_angle((float)Math.Round(position*4096/360)*360/4096);
  min=Math.Min(min,position);max=Math.Max(max,position);
  if(s_autotune_state==AUTOTUNE_SUCCESS || s_autotune_state==AUTOTUNE_FAILED) {finish=now;break;}
 }
 tuning=false;
 Check(finish>0,"Motor-driven tuner did not finish");
 Check(max>104 && min<76,"Motor-driven tuner did not reach both 15-degree offsets: "+min+".."+max);
 Console.WriteLine("PASS full tuner with PWM-driven quantized motor: "+s_autotune_state+", "+finish+" ms, range "+min+".."+max);
}
public static void Run() {
 TunePlantTest();
 Reset(100);s_integrals[0]=200;s_profiled_vels[0]=50;pid_set_target_angle(1,90);Tick();Check(s_integrals[0]<1 && s_profiled_vels[0]<0 && duties[0]<0,"Stored torque delayed reversal");
 TuneTest(false,false);TuneTest(true,false);TuneTest(false,true);s_control_mode=pid_control_mode_t.FOLLOW;resets=0;
 for(int n=0;n<1000;n++)pid_set_mode(pid_control_mode_t.FOLLOW);Check(resets==0,"Repeated mode reset control history");pid_set_mode(pid_control_mode_t.MANUAL);Check(resets==1,"Mode transition failed to reset");
 Reset(90);tuning=true;pid_set_target_angle(1,150);pid_set_mode(pid_control_mode_t.FOLLOW);Check(s_targets[0]==90 && s_control_mode==pid_control_mode_t.MANUAL,"Stream interrupted calibration");tuning=false;
 PlantMove(110,10,2,4);PlantMove(10,110,2,4);PlantMove(350,10,2,4);PlantMove(110,10,5,2);PlantMove(110,10,1,8);
 Move(110,10,10);Move(160,110,110);Move(350,10,370);Move(10,350,-10);Move(90,0,.5f);Move(90,180,180);Move(350,360,359.5f);Move(0,180,180);
 Reset(90);pid_set_target_angle(1,180);Tick();Check(Math.Abs(duties[0])<110,"Large target step bypassed profile");
 Reset(350);pid_set_target_angle(1,10);Tick();float mapped=s_unwrapped_targets[0];
 for(int i=0;i<200;i++) {sensor=wrap_angle(sensor-1);pid_set_target_angle(1,10);Tick();}
 Check(s_unwrapped_targets[0]==mapped,"Repeated target changed destination turn");
 Check(Math.Abs(s_errors[0])<=25.01f,"Tracking lead escaped clamp");
 Console.WriteLine("PASS fixed destination during repeated commands and >180 degree disturbance");
 Reset(0);s_target_initialized[0]=false;s_angle_initialized[0]=false;pid_set_target_angle(1,10);sensor=350;Tick();Check(s_unwrapped_targets[0]==370,"Early command lost");
 Console.WriteLine("PASS command before first sensor reading");
 Reset(90);pid_set_target_angle(1,float.NaN);Check(s_targets[0]==90,"NaN command accepted");
 for(int i=0;i<3000;i++){pid_set_target_angle(1,180);Tick();Check(Math.Abs(duties[0])<=700,"Duty ceiling exceeded");Check(Math.Abs(s_integrals[0])<=200,"Integral ceiling exceeded");}
 Console.WriteLine("PASS invalid target and stalled output/integral bounds");
 Reset(90);s_params.kp=0;s_params.ki=0;s_params.kd=0;s_params.kff=0;s_params.breakaway_fwd=200;s_params.breakaway_rev=250;s_params.coulomb_fwd=30;s_params.coulomb_rev=40;pid_set_target_angle(1,100);
 for(int i=0;i<50;i++) Tick();Check(duties[0]==30,"Forward breakaway did not expire");
 pid_set_target_angle(1,80);for(int i=0;i<500;i++)Tick();Check(duties[0]==-40,"Reverse friction incorrect");
 Console.WriteLine("PASS asymmetric friction and bounded breakaway pulse");
 Reset(90);s_last_actuals[0]=89.8f;Tick();Check(duties[0]!=0,"Deadband suppressed damping while moving");
 Console.WriteLine("PASS deadband velocity gate");
}
}
'@
# Execute production control blocks with C# platform and math adapters.
$split = $prefix.IndexOf('static void Move(')
$combined = $prefix.Substring(0,$split) + $body + $prefix.Substring($split)
Add-Type -TypeDefinition $combined
[PidRegression]::Run()
