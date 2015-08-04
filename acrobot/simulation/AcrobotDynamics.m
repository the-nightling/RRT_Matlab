function [theta1_dd, theta2_dd] = AcrobotDynamics(theta1, theta1_d, theta2, theta2_d, tau)

    %syms q qd qdd;
    %syms m1 I1 l1 lc1 b1;
    %syms m2 I2 l2 lc2 b2;

    m1 = 1;
    m2 = 1;
    l1 = 1;
    l2 = 1;
    lc1 = l1/2;
    lc2 = l2/2;
    Ic1 = (lc1*lc1)/3;
    Ic2 = (lc2*lc2)/3;
    I1 = Ic1+m1*lc1*lc1;
    I2 = Ic2+m2*lc2*lc2;
    b1 = 0.4;
    b2 = 0.4;
    g = 9.8;
    
    q = [theta1; theta2];
    qd = [theta1_d; theta2_d];

    H = [I1 + I2 + m2*l1^2 + 2*m2*l1*lc2*cos(q(2)), I2 + m2*l1*lc2*cos(q(2));
         I2 + m2*l1*lc2*cos(q(2)),                  I2                      ];

    C = [-2*m2*l1*lc2*sin(q(2))*qd(2) + b1, -m2*l1*lc2*sin(q(2))*qd(2);
         m2*l1*lc2*sin(q(2))*qd(1),         b2                        ];

    G = [m1*g*lc1*sin(q(1)) + m2*g*(l1*sin(q(1))+lc2*sin(q(1)+q(2)));
         m2*g*lc2*sin(q(1)+q(2))                                    ];

    B = [0;
         1];
     
    qdd = H\(B*tau - C*qd - G);
    
    theta1_dd = qdd(1);
    theta2_dd = qdd(2);
