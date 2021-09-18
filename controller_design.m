

g = 9.81; % m/s^2
m_c = 1.0; % kg
m_q = 0.2; % kg
l_hat = 0.2; % m
J = m_q*l_hat^2/3;

M = m_c + m_q;

D = M*(m_q*l_hat^2 + J) - (m_q*l_hat)^2;


A = [0,     1,      0,           0;
     0,     0, -m_q*g*l_hat^2/D, 0;
     0,     0,      0,           1;
     0,     0,  M*m_q*g*l_hat/D, 0];

B = [0; (m_q*l_hat^2 + J)/D; 0; -m_q*l_hat/D];

poles = [-1, -2, -3, -4];

K = place(A,B,poles)

