%% Solution Using Symbolic Toolbox (STB) in
%% MATLAB Version 6.0
%%
S=dsolve('Dx1=x2+3,Dx2=-lambda2,Dlambda1=0,Dlambda2=-lambda1,x1(0)=15,x2(0)=20,x1(60)=0,x2(60)=0');
S.x1
S.x2
S.lambda1
S.lambda2

%Plot command is used for which we need to
%% convert the symbolic values to numerical values.
j=1;
for tp=0: .02: 60
t=sym(tp);
x1p(j)=double(subs(S.x1));
%% subs substitutes S.xl to xlp
x2p(j)=double(subs(S.x2));
%% double converts symbolic to numeric
up(j)=-double(subs(S.lambda2));
%% optimal control u = -lambda_2
t1(j)=tp;
j=j+1;
end

figure 1
plot(t1,x1p, 'k' ,t1,x2p, 'k' ,t1,up, 'k:')
xlabel ('t')
% gtext('x_1(t) ')
% gtext (' x_2(t) ')
% gtext ('u(t) ')

figure 2
plot(x1p, x2p, 'k' )
xlabel ('x1')
ylabel ('x2')

figure 3
plot(t1, up*2, 'r-' )
xlabel ('J')
ylabel ('t')