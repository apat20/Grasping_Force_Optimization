T_i = [23;34;12;25];

[m,n] = size(T_i);

T_max = [30;30;30;30];
T_min = [15;15;15;15];

cvx_begin
    cvx_precision high
    variable T(1)
    variable T_i(m,n)
    minimize T
    subject to
        max(T_i) < T;
        T_max >= T_i >= T_min;
cvx_end
