%prob 是1*1, mu是1*3,sigma是3*3,x是1*3
function prob = g_kx(mu,sigma,x)
temp1 = 1/(((2*pi).^(3/2))*(det(sigma).^0.5));
temp2 = -0.5*(x-mu)*inv(sigma)*(x-mu)';
prob = temp1*exp(temp2);

