robot2;

syms q1 q2 q3 q4 q5 q6;
q = [q1 q2 q3 q4 q5 q6];

J = simplify(R.jacob0(q))