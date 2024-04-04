function [K, R, t] = estimate_params(P)
% ESTIMATE_PARAMS computes the intrinsic K, rotation R and translation t from
% given camera matrix P.
% Source 1: RQ decomposition: https://www.uio.no/studier/emner/matnat/its/nedlagte-emner/UNIK4690/v17/forelesninger/lecture_5_2_pose_from_known_3d_points.pdf
% Source 2: use QR to perform RQ: https://math.stackexchange.com/questions/1640695/rq-decomposition
[~,~,V] = svd(P);
c = V(1: 3, end) / V(end, end);
T = [0, 0, 1; 0, 1, 0; 1, 0, 0];
[q, r] = qr((T * P(1: 3, 1: 3))');
q = T * q';
r = T * r' * T;
D = diag(sign(diag(r)));
K = r * D;
R = D * q;
if det(R) < 0
    R = -R;
end
t = - R * c;
