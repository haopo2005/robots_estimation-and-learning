%prob ��1*1, mu��1*3,sigma��3*3,x��1*3
function prob = g_kx(mu,sigma,x)
temp1 = 1/(((2*pi).^(3/2))*(det(sigma).^0.5));
temp2 = -0.5*(x-mu)*inv(sigma)*(x-mu)';
prob = temp1*exp(temp2);

