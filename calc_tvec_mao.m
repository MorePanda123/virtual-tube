% r=0:pos  1:vel  2:acc 3:jerk
function tvec = calc_tvec_mao(t_b,t,n_order,r)
    tvec = zeros(1,n_order+1);
    for i=r+1:n_order+1
        tvec(i) = prod(i-r:i-1)*(t_b+0.2*(t-t_b))^(i-r-1);
    end
end