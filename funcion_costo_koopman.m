function [cost] = funcion_costo_koopman(x, N, X_1, X_K, U, alpha, beta)
                                             
he_koop = [];
he_prediction = [];
%% Matrix system
%% Model Of the system
A = [ x(1), x(2),  x(3),  x(4),  x(5),  x(6),  x(7),  x(8),  x(9),  x(10), x(11), x(12);...
     x(13), x(14), x(15), x(16), x(17), x(18), x(19), x(20), x(21), x(22), x(23), x(24);...
     x(25), x(26), x(27), x(28), x(29), x(30), x(31), x(32), x(33), x(34), x(35), x(36);...
     x(37), x(38), x(39), x(40), x(41), x(42), x(43), x(44), x(45), x(46), x(47), x(48);...
     x(49), x(50), x(51), x(52), x(53), x(54), x(55), x(56), x(57), x(58), x(59), x(60);...
     x(61), x(62), x(63), x(64), x(65), x(66), x(67), x(68), x(69), x(70), x(71), x(72);...
     x(73), x(74), x(75), x(76), x(77), x(78), x(79), x(80), x(81), x(82), x(83), x(84);...
     x(85), x(86), x(87), x(88), x(89), x(90), x(91), x(92), x(93), x(94), x(95), x(96);...
     x(97), x(98), x(99), x(100), x(101), x(102), x(103), x(104), x(105), x(106), x(107), x(108);...
     x(109), x(110), x(111), x(112), x(113), x(114), x(115), x(116), x(117), x(118), x(119), x(120);...
     x(121), x(122), x(123), x(124), x(125), x(126), x(127), x(128), x(129), x(130), x(131), x(132);...
     x(133), x(134), x(135), x(136), x(137), x(138), x(139), x(140), x(141), x(142), x(143), x(144);...
     ];
 
B = [x(145), x(146), x(147);...
     x(148), x(149), x(150);...
     x(151), x(152), x(153);...
     x(154), x(155), x(156);...
     x(157), x(158), x(159);...
     x(160), x(161), x(162);...
     x(163), x(164), x(165);...
     x(166), x(167), x(168);...
     x(169), x(170), x(171);...
     x(172), x(173), x(174);...
     x(175), x(176), x(177);...
     x(178), x(179), x(180);...
     ];

C_a = [eye(6,6), zeros(6,6)];

for k = 1:length(U)

    %% Minimization Koopman
    % %% Lifdted space system
    x_1 = C_a*X_1(:,k);
    x_k = C_a*X_K(:,k);

    Gamma_k = (X_K(:,k));
    Gamma_1 = (X_1(:,k));
    error_koop = Gamma_k  -A*Gamma_1 - B*U(:,k) ;
    
    error_prediction = x_k - C_a*(A*Gamma_1 + B*U(:,k));
    %% Error Vector
    he_koop = [he_koop; error_koop];
    he_prediction = [he_prediction; error_prediction];
    
end
cost = beta*norm(he_koop, 2)^2 + alpha*norm(A, 'fro') + alpha*norm(B, 'fro') + norm(he_prediction, 2)^2;

end