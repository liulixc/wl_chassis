function K = get_k_length(leg_length)
   
    %theta : 摆杆与竖直方向夹角             R   ：驱动轮半径
    %x     : 驱动轮位移                    L   : 摆杆重心到驱动轮轴距离
    %phi   : 机体与水平夹角                LM  : 摆杆重心到其转轴距离
    %T     ：驱动轮输出力矩                 l   : 机体重心到其转轴距离
    %Tp    : 髋关节输出力矩                 mw  : 驱动轮转子质量
    %N     ：驱动轮对摆杆力的水平分量        mp  : 摆杆质量
    %P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量
    %Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量
    %Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量
    %Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量

    syms theta(t) x(t) phi(t) T Tp R L LM l mw mp M Iw Ip IM g  
    syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

    R1=0.075;                         %驱动轮半径
    L1=leg_length/2;                  %摆杆重心到驱动轮轴距离
    LM1=leg_length/2;                 %摆杆重心到其转轴距离
    l1=0.03;                          %机体质心距离转轴距离
    mw1=1.18;                         %驱动轮质量
    mp1=1.50186;                         %杆质量
    M1=12.499;                          %机体质量

   Iw1=mw1*R1^2;                     %驱动轮转动惯量
    Ip1=mp1*((L1+LM1)^2+0.05^2)/12.0; %摆杆转动惯量
   IM1=M1*(0.3^2+0.12^2)/12.0;       %机体绕质心转动惯量

    
    NM = M*diff(x + (L + LM )*sin(theta)-l*sin(phi),t,2);
    N = NM + mp*diff(x + L*sin(theta),t,2);
    PM = M*g + M*diff((L+LM)*cos(theta)+l*cos(phi),t,2);
    P = PM +mp*g+mp*diff(L*cos(theta),t,2);

    eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta,t,2) == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
    eqn3 = IM*diff(phi,t,2) == Tp +NM*l*cos(phi)+PM*l*sin(phi);
    
    eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);

    
    [f1,f2,f3] = solve(eqn10,eqn20,eqn30,f1,f2,f3);

    A=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[theta0,d_theta,x0,d_x,phi0,d_phi]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    A=subs(A,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    A=double(A);
    B=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[T,Tp]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    B=subs(B,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    B=double(B);
    
    % 板凳
%     Q = diag([1 1 40 7 5000 3]); % step1
    
    % 轮腿
%     Q = diag([200 50 40 7 5000 3]); % step2
%     Q = diag([200 1 40 7 5000 3]);
%     Q = diag([100 1 40 7 5000 3]);% step3
%     Q = diag([100 1 40 7 36000 1]);
%     Q = diag([100 1 50 7 36000 1]);

    % 第一轮调参的腿长摆角过大
    %板凳
%     Q = diag([1 1 8 15 1000 1]);
%     Q = diag([500 1 8 15 3000 1]); % 腿不震荡
%     Q = diag([900 2 8 15 5000 1]);
%     Q = diag([900 1 8 15 35000 1]);
      Q = diag([900 1 25 7 35000 1]);

    R = [1.25 0;0 0.25]; %T Tp
    
    K = lqr(A,B,Q,R); % lqr函数返回一个 p x n 的矩阵
  
end