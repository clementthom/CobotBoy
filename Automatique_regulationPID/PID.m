
  clear all %clears variables
  clc %clears console


  pkg load control %load the package to use control functions (eg tf)
  %constant values
  Rmotor = 3.8 %motor resistance
  k = 4.5e-3%motor constant
  Rcap = 10 %resistance used to convert Iout to a voltage to process
  f=6.86e-7 %friction coeff
  L = 1.6e-3 %internal inductance of the motor
  Ufreeload = 1.5 %input voltage = Iout/Rcap when no load is applied to the motor
  Ublocked = 2.6 %input voltage when the motor is blocked (no rotation but torque)


  %transfer function num and denum
  num = [0.03]
  denum = [4.8e-5 1]
  %% Time vector — column for lsim compatibility (the ')
  t = (0:1e-6:0.001)';  % 100 µs step, fine enough for tau = 48 µs --> simulation time
  inputRef  = Ufreeload*ones(length(t), 1); %input step = 1*max


  %transfer function creation (U=5V)
  Hp = tf(num, denum)
  %resistance TF
  Hcap = tf(Rcap, 1); %[Rcap] also works, but better to define it as a TF


  %closed loop without PID
  CLnoPID = feedback(Hp, Hcap)
  outputNoPIDNoDisturb = lsim(CLnoPID, inputRef, t);
  %{
  figure(1); step(CLnoPID); hold on;
  title('Closed-loop response without PID');
  %}


  figure(1); clf;
  plot(t, outputNoPIDNoDisturb,'b');  hold on;
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop response without correction');
  legend('Without PID nor disturbance','Location', 'eastoutside');


  %Very quick profile --> electrical only (since we don't have J)
  %very low current : Hcap creates a very strong feedback compared to the command voltage



  %PID creation
  Kp = 1000%proportional coeff --> we want I=U*Hc*Hp=0.15 at tf --> Hc*Hp =0.15/5=0.03
  Ki = 0%integration coeff
  Kd = 0%derivative coeff
  %only a big proportional (line) gain is useful here (1st order : no peak to moderate) --> won't be the case when we will introduce distrurbances
  %a big Ki also do the trick, but we have a peak over the command for Ki = 50000


  Hc = pid(Kp, Ki, Kd)
  ClwithPIDNoDisturb = feedback(Hp*Hc, Hcap)
  outputWithPIDNoDisturb = lsim(ClwithPIDNoDisturb, inputRef, t);

  %{
  step(ClwithPID, 'Color', 'r'); hold on;
  legend('Without PID', 'With PID, Kp only');


  figure(2); step(ClwithPID); grid on;
  title('Closed-loop response with PID Kp only');
  %}

  figure(2); %clf;
  plot(t, outputNoPIDNoDisturb, 'b');  hold on;
  plot(t, outputWithPIDNoDisturb, 'r');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses with and without proportional correction');
  legend('Without PID', 'With PID Kp only', 'Location', 'eastoutside');


  %we now add a perturbation
  Cblocked = 1.2e-3 %torque delivered by the motor when blocked : resistant torque
  Iblocked = Cblocked/k
  delayBlocked = 0.0005 %0.5 ms after launch, motor blocked
  Ublocked = 2.6 %input command voltage when omega is null --> = 0.26 A * 10 ohms
  inputRef(t >= delayBlocked) = Ublocked; %if condition respected corresponding index = UcommandWhenBlocked


  % Simulate the disturbance as a delayed step
  disturbance = zeros(length(t), 1); %vector of size t and value 0, must be vertical
  disturbance(t >= delayBlocked) = Iblocked; %if condition respected corresponding index = Iblocked


  %creating disturbance current transfer function (TF) --> feedback loop --> see schematics
  Hdisturb = feedback(tf(1,[1]), Hp*Hc*Hcap) %can't do feedback without 2 FTs--> the first one is just 1 though


  %outputs generation
  %recalculation of output with change in command --> inputRef changed
  outputWithPIDDisturbCommand = lsim(ClwithPIDNoDisturb, inputRef, t);
  disturbToAdd = lsim(Hdisturb, disturbance, t) %disturbance to add to the output before FB

  ClwithPIDAndPerturbation = outputWithPIDDisturbCommand+disturbToAdd; %we add the both of them to get the response

  %{
  figure(1), plot(t, ClwithPIDAndPerturbation, 'g'); grid on; hold on;
  legend('Without PID', 'With PID, Kp only','With PID and perturbation, Kp only');
  %}



  figure(3); clf;
  plot(t, outputNoPIDNoDisturb, 'b');  hold on;
  plot(t, outputWithPIDNoDisturb, 'r');
  plot(t, ClwithPIDAndPerturbation,'g');
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses');
  legend('Without PID', 'With PID Kp only', 'With PID + perturbation', 'Location', 'eastoutside');



% we have a current peak superior to the one mentionned in the datasheet (albeit a small one) : need to change PID params

 Kp=1000 %no need to change --> good steady state value
 Ki=0 %we guess that the resistance torque is constant : no perturbation variation
 Kd=0.08 %Kd reduces the current peak, but a too large value can reduce precision at steady state


 Hc = pid(Kp, Ki, Kd)
 ClwithPID = feedback(Hp*Hc, Hcap)
 outputWithNewPIDDisturbCommand = lsim(ClwithPID, inputRef, t);


 disturbance = zeros(length(t), 1); %vector of size t and value 0, must be vertical
 disturbance(t >= delayBlocked) = Iblocked; %if condition respected corresponding index = Iblocked

 Hdisturb = feedback(tf(1,[1]), Hp*Hc*Hcap) %can't do feedback without 2 FTs--> the first one is just 1 though


 disturbToAdd = lsim(Hdisturb, disturbance, t) %disturbance to add to the output before FB

 ClwithNewPIDAndPerturbation = outputWithNewPIDDisturbCommand+disturbToAdd;

 figure(4); clf;
  plot(t, outputNoPIDNoDisturb, 'b');  hold on;
  plot(t, outputWithPIDNoDisturb, 'r');
  plot(t, ClwithPIDAndPerturbation,'g');
  plot(t, ClwithNewPIDAndPerturbation,'y');
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses');
  legend('Without PID', 'With PID', 'With former PID + perturbation', 'With new PID + perturbation', 'Location', 'eastoutside');

  %to focus on the current peak
  figure(5); clf;
  plot(t, ClwithPIDAndPerturbation,    'g'); hold on;
  plot(t, ClwithNewPIDAndPerturbation, 'y');
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Peak zoom — disturbance response');
  legend('Former PID + perturbation', 'New PID + perturbation');

  % Zoom window: centered on delayBlocked, ±10% of total time on x
  % y range: just above 0.26A to capture the peak
  xlim([delayBlocked - 0.01e-3   delayBlocked + 0.01e-3]);
  ylim([0.15   0.30]);   % adjust upper bound if peak exceeds 0.30A


 %we create a model with perturbation and without correction to see the effect on the PID


 disturbance = zeros(length(t), 1); %vector of size t and value 0, must be vertical
 disturbance(t >= delayBlocked) = Iblocked; %if condition respected corresponding index = Iblocked

 Hdisturb = feedback(tf(1,[1]), Hp*Hcap) %can't do feedback without 2 FTs--> the first one is just 1 though

 outputNoPIDWithDisturb = lsim(CLnoPID, inputRef, t);
 disturbToAdd = lsim(Hdisturb, disturbance, t) %disturbance to add to the output before FB

 ClnoPIDAndPerturbation = outputNoPIDWithDisturb+disturbToAdd;

 figure(6); clf;
  plot(t, outputNoPIDNoDisturb, 'b');  hold on;
  plot(t, outputWithPIDNoDisturb, 'r');
  plot(t, ClwithPIDAndPerturbation,'g');
  plot(t, ClwithNewPIDAndPerturbation,'y');
  plot(t, ClnoPIDAndPerturbation,'m'); %color magenta
  xline(delayBlocked, ':k', 'Disturbance on');
  grid on;
  xlabel('Time (s)'); ylabel('Current (A)');
  title('Closed-loop responses');
  legend('Without PID', 'With PID', 'With former PID + perturbation', 'With new PID + perturbation' , 'With perturbation and no PID','Location', 'eastoutside');

