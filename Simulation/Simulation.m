1;
pkg load control

## Function : draw_cart_pendulum()
## ----------------------------------------------------
## Input:   y - State Vector. In case of inverted cart pendulum, the state variables
##              are position of cart x, velocity of cart x_dot, angle of pendulum
##              bob theta wrt vertical and angular velocity theta_dot of pendulum
##              bob.
##
## Purpose: Takes the state vector as input. It draws the inverted cart pendulum in 
##          a 2D plot.
function draw_cart_pendulum(y,m, M, L)
  x = y(1);
  theta = y(3);
  
  W = 1*sqrt(M/5);    # cart width
  H = 0.5*sqrt(M/5);  # cart height 
  wr = 0.2;           # wheel radius
  mr = 0.3*sqrt(m);    # mass radius 
  
  y = wr/2 + H/2;
  w1x = x - 0.9*W/2;
  w1y = 0;
  w2x = x + 0.9*W/2 - wr;
  w2y = 0;
  
  px = x - L*sin(theta);
  py = y - L*cos(theta);
   
  hold on;
  clf;
  axis equal;
  rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1])
  rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
  rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
  
  line ([-10 10], [0 0], "linestyle", "-", "color", "k");
  line ([x px], [y py], "linestyle", "-", "color", "k");
  rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.1 0.1 1])
  
  xlim([-10 10]);
  ylim([-2 3]);
  set(gcf, 'Position', [200 300 1000 400]);
  drawnow
  hold off
endfunction
## Function : cart_pendulum_dynamics()
## ----------------------------------------------------
## Input:   y - State Vector. In case of inverted cart pendulum, the state variables
##              are position of cart x, velocity of cart x_dot, angle of pendulum
##              bob theta wrt vertical and angular velocity theta_dot of pendulum
##              bob.
##          m - Mass of pendulum bob
##          M - Mass of cart
##          L - Length of cart
##          g  - Acceleration due to gravity
##          u  - Input to the system. Input is the horizontal force acting on the cart.
##
## Output:  dy -  Derivative of State Vector.
##
## Purpose: Calculates the value of the vector dy according to the equations which 
##          govern this system.
function dy = cart_pendulum_dynamics(y, m, M, L, Iw, Ib, r,  g,  u)
  sinT = sin(pi - y(3));
  cosT = cos(pi - y(3));
  
  k = (m.*m.*L.*L)/(m.*L.*L + Ib);
  
  dy(1,1) = y(2);
  dy(2,1) = (1./(m + 2.*M + 2.*Iw./(r.*r) - k.*cosT.*cosT)).*(-k.*g.*cosT.*sinT + m.*L.*y(4).*y(4).*sinT + (1./r + (k.*cosT)./(m.*L)).*u);
  dy(3,1) = y(4);
  dy(4,1) = (m.*g.*L.*sinT - m.*L.*dy(2,1).*cosT - u)./(m.*L.*L + Ib);
endfunction

## Function : sim_cart_pendulum()
## ----------------------------------------------------
## Input:   m - Mass of pendulum bob
##          M - Mass of cart
##          L - Length of cart
##          g  - Acceleration due to gravity
##          y0 - Initial Condition of system
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of cart pendulum system without 
##          any external input (u).
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0
function [t,y] = sim_cart_pendulum(m, M, L, Iw, Ib, r, g, y0)
  tspan = 0:0.1:10;                  ## Initialise time step           
  u = 0;            ## No Input
  [t,y] = ode45(@(t,y)cart_pendulum_dynamics(y, m, M, L, Iw, Ib, r, g, u),tspan,y0); ## Solving the differential equation   
endfunction

## Function : cart_pendulum_AB_matrix()
## ----------------------------------------------------
## Input:   m - Mass of pendulum bob
##          M - Mass of cart
##          L - Length of cart
##          g  - Acceleration due to gravity
##
## Output:  A - A matrix of system
##          B - B matrix of system
##          
## Purpose: Declare the A and B matrices in this function.
function [A, B] = cart_pendulum_AB_matrix(m , M, L, Iw, Ib, r, g)
  
  k = (m*m*L*L)/(m*L*L + Ib);
  
  A = [0 1 0 0;
       0 0 (-k*g)/(m + 2*M + 2*Iw/(r*r) - k) 0;
       0 0 0 1; 
       0 0 (k*g*(m + 2*M + 2*Iw/(r*r)))/(m*L*(m + 2*M + 2*Iw/(r*r) - k)) 0];
  B = [0; 
       (1/r + k/(m*L))/(m + 2*M + 2*Iw/(r*r) - k);
       0;
       (-k/(m*m*L*L))-(k/(m*L))*(1/r + k/(m*L))*(1/(m + 2*M + 2*Iw/(r*r) - k))];  
endfunction

## Function : pole_place_cart_pendulum()
## ----------------------------------------------------
## Input:   m - Mass of pendulum bob
##          M - Mass of cart
##          L - Length of cart
##          g  - Acceleration due to gravity
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of inverted cart pendulum with 
##          external input using the pole_placement controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using Pole Placement Technique.
function [t,y] = pole_place_cart_pendulum(m, M, L, Iw, Ib, r, g, y_setpoint, y0)
  [A, B] = cart_pendulum_AB_matrix(m, M, L, Iw, Ib, r, g);
  eigs = [-0.00005, -0.0001, -0.5, -0.005];         ## Initialise desired eigenvalues
  K = place(A, B, eigs)   ## Calculate K matrix for desired eigenvalues
  tspan = 0:0.1:40;
  [t,y] = ode45(@(t,y)cart_pendulum_dynamics(y, m, M, L, Iw, Ib, r, g, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : lqr_cart_pendulum()
## ----------------------------------------------------
## Input:   m - Mass of pendulum bob
##          M - Mass of cart
##          L - Length of cart
##          g  - Acceleration due to gravity
##          y_setpoint - Reference Point
##          y0 - Initial Condition
##
## Output:  t - Timestep
##          y - Solution array
##          
## Purpose: This function demonstrates the behavior of inverted cart pendulum with 
##          external input using the LQR controller
##          This integrates the system of differential equation from t0 = 0 to 
##          tf = 10 with initial condition y0 and input u = -Kx where K is
##          calculated using LQR Controller.
function [K] = lqr_cart_pendulum(m, M, L, Iw, Ib, r, g, y_setpoint, y0)
  [A,B] = cart_pendulum_AB_matrix(m, M, L, Iw, Ib, r, g);   ## Initialize A and B matrix
  Q = [2 0 0 0;            ## Initialise Q matrix
       0 2 0 0;
       0 0 6 0;
       0 0 0 6];            
  R = 1000;                   ## Initialise R 
  #csys = ss(A, B);
  #dsys = c2d(csys, 4*1e-3);
  #[G, X, L] = dlqr(dsys, Q, R);
  #disp(G);
  K = lqr(A, B, Q, R);
  disp(K);
  ## Calculate K matrix from A,B,Q,R matrices
  #tspan = 0:0.1:40;
  #[t,y] = ode45(@(t,y)cart_pendulum_dynamics(y, m, M, L, Iw, Ib, r, g, -K*(y-y_setpoint)),tspan,y0);
endfunction

## Function : cart_pendulum_main()
## ----------------------------------------------------
## Purpose: Used for testing out the various controllers by calling their 
##          respective functions and observing the behavior of the system. Constant
##          parameters like mass of cart, mass of pendulum bob etc are declared here.
function cart_pendulum_main()
  m = 1083*1e-3;
  M = 33*1e-3;
  L = 7.72*1e-2;
  Iw = 133.85*1e-7;
  Ib = 79275.07*1e-7;
  r = 3.3*1e-2;
  g = 9.80;
  y0 = [0; 0; pi + 0.3; 0];
  y_setpoint = [0; 0; pi; 0];
##[t,y] = sim_cart_pendulum(m, M, L, Iw, Ib, r, g, y0);
##[t,y] = pole_place_cart_pendulum(m, M, L, Iw, Ib, r, g, y_setpoint, y0)
[K] = lqr_cart_pendulum(m, M, L, Iw, Ib, r, g, y_setpoint, y0);
  ##for k = 1:length(t) 
    ##draw_cart_pendulum(y(k, :), 1, 5, 2);  
  ##endfor
  
endfunction

