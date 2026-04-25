
  clear all %clears variables
  clc %clears console


  pkg load control %load the package to use control functions (eg tf)
  %constant values
  Rmotor = 3.8 %motor resistance
  k = 4.5e-3%motor constant
  Rcap = 10 %resistance used to convert Iout to a voltage to process
  Ufreeload = 1.5 %input voltage = Iout/Rcap when no load is applied to the motor
  Ublocked = 2.6 %input voltage when the motor is blocked (no rotation but torque)


  %transfer function num and denum
  num = [0.03]
  denum = [4.8e-5 1]
  %% Time vector — column for lsim compatibility (the ')
  t = (0:1e-5:0.005)';              % 100 µs step, fine enough for tau = 48 µs --> simulation time
  inputRef  = Ufreeload*ones(length(t), 1); %input step = 1*max



  %transfer function creation (U=5V)
  Hp = tf(num, denum)
  %resistance TF
  Hcap = tf(R, 1); %[H] also works, but better to define it as a TF

%{
  %closed loop without PID
  CLnoPID = feedback(Hp, Hcap)
  outputNoPID = lsim(CLnoPID, inputRef, t);
  %{
  figure(1); step(CLnoPID); hold on;
  title('Closed-loop response without PID');
  %}

  figure(1); clf;
  plot(t, outputNoPID,'b');  hold on;
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop response without correction');
  legend('Without PID');


  %Very quick profile --> electrical only (since we don't have J)



  %PID creation
  Kp = 1.13%proportional coeff --> we want I=U*Hc*Hp=0.15 at tf --> Hc*Hp =0.15/5=0.03
  Ki = 0%integration coeff
  Kd = 0%derivative coeff


  Hc = pid(Kp, Ki, Kd)
  ClwithPID = feedback(Hp*Hc, Hcap)
  outputWithPID = lsim(ClwithPID, inputRef, t);

  %{
  step(ClwithPID, 'Color', 'r'); hold on;
  legend('Without PID', 'With PID, Kp only');


  figure(2); step(ClwithPID); grid on;
  title('Closed-loop response with PID Kp only');
  %}

  figure(2); clf;
  plot(t, outputNoPID, 'b');  hold on;
  plot(t, outputWithPID, 'r');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses with and without proportional correction');
  legend('Without PID', 'With PID Kp only');


  %we now add a perturbation
  Cblocked = 1.2e-3 %torque delivered by the motor when blocked : resistant torque
  Iblocked = Cblocked/k
  delayBlocked = 0.001 %50 ms after launch, motor blocked
  Ublocked = 1 %input voltage when omega is null
  inputRef(t >= delayBlocked) = Ublocked; %if condition respected corresponding index = Iblocked


  % Simulate the disturbance as a delayed step
  disturbance = zeros(length(t), 1); %vector of size t and value 0, must be vertical
  disturbance(t >= delayBlocked) = Iblocked; %if condition respected corresponding index = Iblocked


  %creating disturbance current transfer function (TF) --> feedback loop --> see schematics
  Hdisturb = feedback(tf(1,[1]), Hp*Hc*Hcap) %can't do feedback without 2 FTs--> the first one is just 1 though


  %outputs generation
  disturbToAdd = lsim(Hdisturb, disturbance, t) %disturbance to add to the output before FB

  ClwithPIDAndPerturbation = outputWithPID+disturbToAdd; %we add the both of them to get the response

  %{
  figure(1), plot(t, ClwithPIDAndPerturbation, 'g'); grid on; hold on;
  legend('Without PID', 'With PID, Kp only','With PID and perturbation, Kp only');
  %}


  figure(3); clf;
  plot(t, outputNoPID, 'b');  hold on;
  plot(t, outputWithPID, 'r');
  plot(t, ClwithPIDAndPerturbation,'g');
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses');
  legend('Without PID', 'With PID Kp only', 'With PID + perturbation');

% we have a current peak superior to the one mentionned in the datasheet : need to change PID params

 Kp=1.13 %I was at 0.27>0.26 A --> motor can be damaged --> now value = 0.26
 Ki=0 %we guess that the resistance torque is constant : no perturbation variation
 Kd=6e-5% needs to be extremely small, otherwise the initial current is too strong; we want a current call inferior to the nominal value
 %Kd reduces the current peak



 Hc = pid(Kp, Ki, Kd)
 ClwithPID = feedback(Hp*Hc, Hcap)
 outputWithPID = lsim(ClwithPID, inputRef, t);


 disturbance = zeros(length(t), 1); %vector of size t and value 0, must be vertical
 disturbance(t >= delayBlocked) = Iblocked; %if condition respected corresponding index = Iblocked

 Hdisturb = feedback(tf(1,[1]), Hp*Hc*Hcap) %can't do feedback without 2 FTs--> the first one is just 1 though


 disturbToAdd = lsim(Hdisturb, disturbance, t) %disturbance to add to the output before FB

 ClwithPIDAndPerturbation = outputWithPID+disturbToAdd;
Iblocked

 figure(4); clf;
  plot(t, outputNoPID, 'b');  hold on;
  plot(t, outputWithPID, 'r');
  plot(t, ClwithPIDAndPerturbation,'g');
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses');
  legend('Without PID', 'With PID', 'With PID + perturbation');

%{
 %we create a model with perturbation and without correction



 disturbance = zeros(length(t), 1); %vector of size t and value 0, must be vertical
 disturbance(t >= delayBlocked) = Iblocked; %if condition respected corresponding index = Iblocked

 Hdisturb = feedback(tf(1,[1]), Hp*Hcap) %can't do feedback without 2 FTs--> the first one is just 1 though


 disturbToAdd = lsim(Hdisturb, disturbance, t) %disturbance to add to the output before FB

 ClwithPIDAndPerturbation = outputNoPID+disturbToAdd;
Iblocked

 figure(5); clf;
  plot(t, outputNoPID, 'b');  hold on;
  plot(t, outputWithPID, 'r');
  plot(t, ClwithPIDAndPerturbation,'g');
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses');
  legend('Without PID', 'With PID', 'With PID + perturbation');
%}
%}
