#include "MinimumJerkInterpolation.h"
#include <vector>

MinimumJerkInterpolation::MinimumJerkInterpolation()
{
}

// coefficient: whenever the acceleration and velocities are same is not suitable to use infinty and the expected velocity and acceleration will not be  continues
// note: that the coeeficents are in format: a1(x-x0)^5+a2*(x-x0)^4+.....+a0
// coefficient1 is suitable for all case. note: that the coeeficents are in format: a1x^5+a2*x^4.......+a0

MatrixXd MinimumJerkInterpolation::Coefficient(MatrixXd time, MatrixXd p, MatrixXd dp, MatrixXd ddp)
{
    // check size of p dp ddp time
    int n = length(time) - 1; // if we have 2 time interval then n is equals 2
    if (n != (length(p) - 1))
    {
        cout << "Size of Position Error!";
        MatrixXd temp;
        return temp;
    }
    if (n != (length(dp) - 1))
    {
        cout << "Size of Velocity Error!";
        MatrixXd temp;
        return temp;
    }
    if (n != (length(ddp) - 1))
    {
        cout << "Size of Acceleration Error!";
        MatrixXd temp;
        return temp;
    }
    // diffrence between two time interval less than zero
    if (diff(time).minCoeff() <= 0)
    {
        cout << "Time Error!";
        MatrixXd temp;
        return temp;
    }

    // define matrix A, H, M, F
    MatrixXd A = MatrixXd::Zero(6 * n, 6 * n);
    MatrixXd H = MatrixXd::Zero(6 * n, 6 * n);
    MatrixXd M = MatrixXd::Zero(6 * n, 3 * (n + 1));

    for (int i = 0; i < n; i++)
    {
        double ti = time(0, i);
        double tf = time(0, i + 1);
        MatrixXd Ai(6, 6);
        Ai << (6 / (-1 * pow(tf, 5) + 5 * pow(tf, 4) * ti - 10 * pow(tf, 3) * pow(ti, 2) + 10 * pow(tf, 2) * pow(ti, 3) - 5 * tf * pow(ti, 4) + pow(ti, 5))), -3 / (pow(tf, 4) - 4 * pow(tf, 3) * ti + 6 * pow(tf, 2) * pow(ti, 2) - 4 * tf * pow(ti, 3) + pow(ti, 4)), -1 / (2 * (pow(tf, 3) - 3 * pow(tf, 2) * ti + 3 * tf * pow(ti, 2) - pow(ti, 3))), -6 / (-1 * pow(tf, 5) + 5 * pow(tf, 4) * ti - 10 * pow(tf, 3) * pow(ti, 2) + 10 * pow(tf, 2) * pow(ti, 3) - 5 * tf * pow(ti, 4) + pow(ti, 5)), -3 / (pow(tf, 4) - 4 * pow(tf, 3) * ti + 6 * pow(tf, 2) * pow(ti, 2) - 4 * tf * pow(ti, 3) + pow(ti, 4)), 1 / (2 * (pow(tf, 3) - 3 * pow(tf, 2) * ti + 3 * tf * pow(ti, 2) - pow(ti, 3))),
            15 / (pow(tf, 4) - 4 * pow(tf, 3) * ti + 6 * pow(tf, 2) * pow(ti, 2) - 4 * tf * pow(ti, 3) + pow(ti, 4)), (8 / (pow(tf, 3) - 3 * pow(tf, 2) * ti + 3 * tf * pow(ti, 2) - pow(ti, 3))), 3 / (2 * (pow(tf, 2) - 2 * tf * ti + pow(ti, 2))), -15 / (pow(tf, 4) - 4 * pow(tf, 3) * ti + 6 * pow(tf, 2) * pow(ti, 2) - 4 * tf * pow(ti, 3) + pow(ti, 4)), 7 / (pow(tf, 3) - 3 * pow(tf, 2) * ti + 3 * tf * pow(ti, 2) - pow(ti, 3)), -1 / (pow(tf, 2) - 2 * tf * ti + pow(ti, 2)),
            -10 / (pow(tf, 3) - 3 * pow(tf, 2) * ti + 3 * tf * pow(ti, 2) - pow(ti, 3)), -6 / (pow(tf, 2) - 2 * tf * ti + pow(ti, 2)), -3 / (2 * (tf - ti)), 10 / (pow(tf, 3) - 3 * pow(tf, 2) * ti + 3 * tf * pow(ti, 2) - pow(ti, 3)), -4 / (pow(tf, 2) - 2 * tf * ti + pow(ti, 2)), 1 / (2 * (tf - ti)),
            0, 0, 0.5, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            1, 0, 0, 0, 0, 0;

        MatrixXd Hi(6, 6);
        Hi << (400 * pow((tf - ti), 7)) / 7, 40 * pow((tf - ti), 6), 24 * pow((tf - ti), 5), 10 * pow((tf - ti), 4), 0, 0,
            40 * pow((tf - ti), 6), (144 * pow((tf - ti), 5)) / 5, 18 * pow((tf - ti), 4), 8 * pow((tf - ti), 3), 0, 0,
            24 * pow((tf - ti), 5), 18 * pow((tf - ti), 4), 12 * pow((tf - ti), 3), 6 * pow((tf - ti), 2), 0, 0,
            10 * pow((tf - ti), 4), 8 * pow((tf - ti), 3), 6 * pow((tf - ti), 2), 4 * tf - 4 * ti, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

        A.block(6 * i, 6 * i, 6, 6) = Ai;
        H.block(6 * i, 6 * i, 6, 6) = Hi;

        M.block(6 * i, 3 * i, 6, 6) = MatrixXd::Identity(6, 6);
    }
    MatrixXd F;
    F = (A * M).transpose() * H * (A * M);
    // PrintArray(F,"F");

    // define matrix x as [p;dp;ddp]
    MatrixXd tempx(p.rows() + dp.rows() + ddp.rows(), p.cols());
    tempx.block(0, 0, p.rows(), p.cols()) = p;
    tempx.block(p.rows(), 0, dp.rows(), dp.cols()) = dp;
    tempx.block(p.rows() + dp.rows(), 0, ddp.rows(), ddp.cols()) = ddp;

    // For reshaping the matrix
    Map<MatrixXd> x_temp(tempx.data(), 3 * (n + 1), 1);
    MatrixXd x = x_temp;

    // finding inf
    vector<int> u = findIsinf(x, true);
    vector<int> v = findIsinf(x, false);

    if (u.size() != 0)
    {
        MatrixXd Fuu(u.size(), u.size());
        MatrixXd Fvv(v.size(), v.size());
        MatrixXd Fuv(u.size(), v.size());
        MatrixXd Fvu(v.size(), u.size());

        for (int i = 0; i < u.size(); i++)
        {
            for (int j = 0; j < u.size(); j++)
                Fuu(i, j) = F(u.at(i), u.at(j));
        }

        for (int i = 0; i < v.size(); i++)
        {
            for (int j = 0; j < v.size(); j++)
                Fvv(i, j) = F(v.at(i), v.at(j));
        }

        for (int i = 0; i < u.size(); i++)
        {
            for (int j = 0; j < v.size(); j++)
                Fuv(i, j) = F(u.at(i), v.at(j));
        }

        for (int i = 0; i < v.size(); i++)
        {
            for (int j = 0; j < u.size(); j++)
                Fvu(i, j) = F(v.at(i), u.at(j));
        }

        MatrixXd G(Fuu.rows() + Fvu.rows(), Fuu.cols() + Fuv.cols());
        G.block(0, 0, Fuu.rows(), Fuu.cols()) = Fuu;
        G.block(0, Fuu.rows(), Fuv.rows(), Fuv.cols()) = Fuv;
        G.block(Fuu.rows(), 0, Fvu.rows(), Fvu.cols()) = Fvu;
        G.block(Fuu.rows(), Fvu.cols(), Fvv.rows(), Fvv.cols()) = Fvv;

        MatrixXd xv = MatrixIndex(x, v);

        MatrixXd b = 0.5 * (xv.transpose() * Fvu + xv.transpose() * Fuv.transpose());
        MatrixXd C = 0.5 * (Fuu.transpose() + Fuu);

        // replace inf element of x with x_inf
        MatrixXd x_inf = -1 * C.inverse() * b.transpose();
        x = AddtoIndexes(x, x_inf, u);
    }
    MatrixXd c = A * M * x; // size = 6n x 1
    // PrintArray(A,"A");
    // PrintArray(M,"M");
    // PrintArray(x,"x");
    Map<MatrixXd> coef_temp(c.data(), 6, 10); // up to ten time interval is possible
    MatrixXd coef = coef_temp;
    return coef.transpose();
}

MatrixXd MinimumJerkInterpolation::Coefficient1(MatrixXd xp, MatrixXd ord, MatrixXd con, double fig_res)
{
    int n = xp.cols();
    if ((xp.rows() != 1) || (xp.rows() != 1) || (ord.rows() != 1) || (ord.cols() != (n - 1)) || (con.rows() != 3) || (con.cols() != n))
    {
        cout << "Size Error!";
        MatrixXd temp;
        return temp;
    }
    MatrixXd conCol1 = isinf(con.col(0));
    MatrixXd conColn = isinf(con.col(n - 1));

    if (!conCol1.isApprox(conColn))
    {
        cout << "Con Matrix is Wrong!";
        MatrixXd temp;
        return temp;
    }
    MatrixXd c1temp1 = con.col(0);
    MatrixXd c1temp2 = con.col(n - 1);
    MatrixXd c1temp(3, 2);
    c1temp << c1temp1, c1temp2;
    //  MatrixXd c1 << isinf(con.block(0,0,con.rows(),n));
    MatrixXd c1 = isinf(c1temp);
    MatrixXd c2 = isinf(con.block(0, 1, con.rows(), con.cols() - 2));

    MatrixXd c3 = isnan(c1temp);
    MatrixXd c4 = isnan(con.block(0, 1, con.rows(), con.cols() - 2));

    MatrixXd::Index maxIndex;
    double connum;
    connum = c1.sum() / 2 + 6 - c1.sum() - c3.sum() + c2.sum() + 2 * (3 * (n - 2) - c2.sum() - c4.sum()); // 2*(3*(n-2)-sum(c2(:))-sum(c4(:)));
    //    if(c2.cols() != 0 && c2.rows() != 0){
    //        connum = connum + c2.colwise().sum().maxCoeff(&maxIndex)+
    //                2*(3*(n-2)-c2.colwise().sum().maxCoeff(&maxIndex));
    //    }
    //    if(c2.cols() != 0 && c2.rows() != 0)
    //        connum = connum - 2*c4.colwise().sum().maxCoeff(&maxIndex);
    if (connum > (ord.sum() + length(ord)))
    {
        cout << "Ord array does not match the constraints! Increase the ord array";
        MatrixXd temp;
        return temp;
    }

    MatrixXd A = MatrixXd::Zero(connum, (ord.sum() + length(ord)));
    MatrixXd C = MatrixXd::Zero(connum, 1);
    // ord=ord-MatrixXd::Ones(1,ord.cols());
    MatrixXd temp_ord(1, 1 + ord.cols());
    temp_ord.block(0, 0, 1, 1) = MatrixXd::Zero(1, 1);
    temp_ord.block(0, 1, 1, ord.cols()) = ord;
    ord = temp_ord;

    int count = -1;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if ((con(j, i) != INFINITY) && (con(j, i) != NAN))
            {
                if (i == 0)
                {
                    ++count;
                    C(count, 0) = con(j, i);
                    //                   double m=SUM1(0,i,ord);
                    //                   double n=SUM1(0,i+1,ord);
                    A.block(count, SUM1(0, i, ord) + i, 1, SUM1(0, i + 1, ord) + i + 1) = PMaker(xp(i)).block(j, 0, 1, ord(0, i + 1) + 1);
                    //                    qDebug("Salam");
                }
                else if (i == (n - 1))
                {
                    ++count;
                    C(count, 0) = con(j, i);
                    MatrixXd MM = PMaker(xp(i)).block(j, 0, 1, ord(0, i) + 1);
                    A.block(count, SUM1(0, i - 1, ord) + i - 1, 1, (SUM1(0, i, ord) + i - 1) - (SUM1(0, i - 1, ord) + i - 1) + 1) = MM; //= PMaker(xp(i)).block(j,0,1,ord(0,i)+1);
                }
                else
                {
                    ++count;
                    C(count, 0) = con(j, i);
                    A.block(count, SUM1(0, i - 1, ord) + i - 1, 1, (SUM1(0, i, ord) + i - 1) - (SUM1(0, i - 1, ord) + i - 1) + 1) = PMaker(xp(i)).block(j, 0, 1, ord(0, i) + 1);
                    ++count;
                    C(count, 0) = con(j, i);

                    A.block(count, SUM1(0, i, ord) + i, 1, (SUM1(0, i + 1, ord) + i) - (SUM1(0, i, ord) + i) + 1) = PMaker(xp(i)).block(j, 0, 1, ord(0, i + 1) + 1);
                }
            }
            else if (con(j, i) == INFINITY)
            {
                if (i == 0)
                {
                    ++count;
                    C(count, 0) = 0;

                    A.block(count, SUM1(0, 0, ord), 1, (SUM1(0, 1, ord)) - (SUM1(0, 0, ord)) + 1) = -1 * PMaker(xp(0)).block(j, 0, 1, ord(0, 2) + 1);
                    A.block(count, SUM1(0, n - 2, ord) + n - 2, 1, (SUM1(0, n - 1, ord) + n - 2) - (SUM1(0, n - 2, ord) + n - 2) + 1) = PMaker(xp(n - 1)).block(j, 0, 1, ord(0, n - 1) + 1);
                }
                else if (i != (n - 1))
                {
                    ++count;
                    C(count, 0) = 0;

                    A.block(count, SUM1(0, i - 1, ord) + i - 1, 1, (SUM1(0, i, ord) + i - 1) - (SUM1(0, i - 1, ord) + i - 1) + 1) = PMaker(xp(i)).block(j, 0, 1, ord(0, i) + 1);
                    A.block(count, SUM1(0, i, ord) + i, 1, (SUM1(0, i + 1, ord) + i) - (SUM1(0, i, ord) + i) + 1) = -1 * PMaker(xp(i)).block(j, 0, 1, ord(0, i + 1) + 1);
                }
            }
        }
    }

    MatrixXd B = A.colPivHouseholderQr().solve(C);

    MatrixXd polycoef = MatrixXd::Zero(ord.maxCoeff() + 1, n - 1);

    for (int i = 0; i < n - 1; i++)
    {
        int colNum1 = ord.block(0, 0, 1, i + 1).rowwise().sum().maxCoeff(&maxIndex) + i;
        int colNum2 = ord.block(0, 0, 1, i + 2).rowwise().sum().maxCoeff(&maxIndex) + i;
        Map<MatrixXd> b_temp(B.data(), B.rows() * B.cols(), 1);
        MatrixXd x = b_temp;
        polycoef.block(0, i, ord(i + 1) + 1, 1) = x.block(colNum1, 0, colNum2 - colNum1 + 1, 1);
    }

    polycoef = polycoef.colwise().reverse().eval();
    return polycoef;
}

double MinimumJerkInterpolation::SUM1(int first, int last, MatrixXd Mat)
{

    // note the first and last should be started from 0
    int row = Mat.rows();
    int col = Mat.cols();
    double sum = 0;
    div_t division = div(first + 1, row);
    int colsum = division.quot; // note index of colsum start from 0
    int rowsum;
    if (division.rem == 0)
    {
        rowsum = row - 1;
        colsum = colsum - 1;
    }
    else
    {
        rowsum = division.rem - 1; // note index of rowsum start from 0
    }
    for (int var = 0; var <= (last - first); var++)
    {
        sum = Mat(rowsum, colsum) + sum;
        if (rowsum == (row - 1))
        {
            colsum = colsum + 1;
            rowsum = 0;
        }
        else
        {
            rowsum = rowsum + 1;
        }
    }

    return sum;
}

MatrixXd MinimumJerkInterpolation::PMaker(double t)
{
    MatrixXd result(3, 21);
    result << 1, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6),
        pow(t, 7), pow(t, 8), pow(t, 9), pow(t, 10), pow(t, 11), pow(t, 12), pow(t, 13),
        pow(t, 14), pow(t, 15), pow(t, 16), pow(t, 17), pow(t, 18), pow(t, 19), pow(t, 20),
        0, 1, 2 * t, 3 * pow(t, 2), 4 * pow(t, 3), 5 * pow(t, 4), 6 * pow(t, 5), 7 * pow(t, 6),
        8 * pow(t, 7), 9 * pow(t, 8), 10 * pow(t, 9), 11 * pow(t, 10), 12 * pow(t, 11), 13 * pow(t, 12), 14 * pow(t, 13),
        15 * pow(t, 14), 16 * pow(t, 15), 17 * pow(t, 16), 18 * pow(t, 17), 19 * pow(t, 18), 20 * pow(t, 19),
        0, 0, 2, 6 * pow(t, 1), 12 * pow(t, 2), 20 * pow(t, 3), 30 * pow(t, 4), 42 * pow(t, 5),
        56 * pow(t, 6), 72 * pow(t, 7), 90 * pow(t, 8), 110 * pow(t, 9), 132 * pow(t, 10), 156 * pow(t, 11), 182 * pow(t, 12),
        210 * pow(t, 13), 240 * pow(t, 14), 272 * pow(t, 15), 306 * pow(t, 16), 342 * pow(t, 17), 380 * pow(t, 18);
    return result;
}

MatrixXd MinimumJerkInterpolation::isinf(MatrixXd in)
{
    MatrixXd temp(in.rows(), in.cols());
    for (int i = 0; i < in.rows(); i++)
    {
        for (int j = 0; j < in.cols(); j++)
            temp(i, j) = (in(i, j) == INFINITY) ?: 0;
    }
    return temp;
}

MatrixXd MinimumJerkInterpolation::isnan(MatrixXd in)
{
    MatrixXd temp(in.rows(), in.cols());
    for (int i = 0; i < in.rows(); i++)
    {
        for (int j = 0; j < in.cols(); j++)
            temp(i, j) = (in(i, j) == NAN) ?: 0;
    }
    return temp;
}

MatrixXd MinimumJerkInterpolation::AddtoIndexes(MatrixXd in, MatrixXd val, vector<int> indexes)
{
    for (int i = 0; i < indexes.size(); i++)
        in(indexes.at(i), 0) = val(i);
    return in;
}

MatrixXd MinimumJerkInterpolation::MatrixIndex(MatrixXd in, vector<int> indexes)
{
    MatrixXd result(indexes.size(), 1);
    for (int i = 0; i < indexes.size(); i++)
    {
        result(i, 0) = in(indexes[i], 0);
    }
    return result;
}

int MinimumJerkInterpolation::length(MatrixXd in)
{
    return max(in.rows(), in.cols());
}

vector<int> MinimumJerkInterpolation::findIsinf(MatrixXd in, bool inf)
{
    vector<int> resultInf;
    vector<int> resultNotInf;
    for (int i = 0; i < in.rows(); i++)
    {
        if (in(i, 0) == INFINITY)
            resultInf.push_back(i);
        else
            resultNotInf.push_back(i);
    }
    if (inf)
        return resultInf;
    else
        return resultNotInf;
}
/*
void MinimumJerkInterpolation::PrintArray(MatrixXd array,string filename){
    ofstream file(filename.toLatin1() + ".csv");
    if(file.is_open()){
        for(int i = 0;i < array.rows();i++){
            for(int j = 0;j < array.cols();j++)
                file << array(i,j) << ",";
            file << "\r" << endl;
        }
    }
}
*/
MatrixXd MinimumJerkInterpolation::diff(MatrixXd E)
{
    MatrixXd E1 = E.block(0, 0, E.rows(), E.cols() - 1);
    MatrixXd E2 = E.block(0, 1, E.rows(), E.cols() - 1);
    return (E2 - E1);
}
// for generated coefs of current interval (Coef.row(i)--- 1 x 6) calculate amount of X, V and A
MatrixXd MinimumJerkInterpolation::GetAccVelPos(MatrixXd Coef, double time, double ti, int PolynomialOrder)
{
    int PolyNomialDegree = PolynomialOrder;
    MatrixXd T(PolyNomialDegree + 1, 1);
    T.fill(0);
    MatrixXd Diag(PolyNomialDegree + 1, PolyNomialDegree + 1);
    Diag.fill(0);
    for (int var = 0; var < PolyNomialDegree + 1; var++)
    {
        T(var, 0) = pow((time - ti), PolyNomialDegree - var);
        if (var != 0)
        {
            Diag.diagonal(1)(var - 1, 0) = PolyNomialDegree - var + 1;
        }
    }
    // Diag is a diagonal matrix with diagonal value as 6, 5, 4, 3, 2, 1
    MatrixXd x = Coef * T;
    double X = x(0, 0);

    MatrixXd v = Coef * Diag * T;
    double V = v(0, 0);

    MatrixXd a = Coef * Diag * Diag * T;
    double A = a(0, 0);

    MatrixXd Output(1, 3);
    Output << X, V, A;
    return Output;
}
