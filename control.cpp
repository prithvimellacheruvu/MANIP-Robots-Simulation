#include <robot_init.h>

using namespace std;

int main(int argc, char ** argv)
{
    // initialize robot class and get DOF's
    auto robot = ecn::initRobot(argc, argv, 100);
    const unsigned n = robot->getDofs();

    // robot properties
    const vpColVector vMax = robot->vMax();
    const vpColVector aMax = robot->aMax();

    // main variables
    vpColVector q(n);               // joint position
    vpPoseVector p;                 // operational pose
    vpPoseVector pe;                // error pose
    vpColVector qCommand(n);        // joint position setpoint
    vpColVector vCommand(n);        // joint velocity setpoint

    vpMatrix J;
    vpHomogeneousMatrix M;          // current pose
    vpHomogeneousMatrix M0, Md, Mi; // previous, final and current desired poses
    vpPoseVector pd;                // desired pose
    vpColVector v;                  // desired operational velocity
    vpColVector ve(6);                 // final error velocity

    // TODO declare other variables if needed
    vpColVector q0(n), qf(n);        // joint position setpoint for initial and final poses
    double t, t0, tf;

    // main control loop
    while(robot->ok())
    {
        // current time
        t = robot->time();

        // update desired pose if has changed
        if(robot->newRef())
        {
            Md = robot->Md();
            M0 = robot->M0();
            pd.buildFrom(Md);
            t0 = t;
        }


        // get current joint positions
        q = robot->jointPosition();
        cout << "Current joint position : " << q.t() << endl;

        // Direct Geometry for end-effector
        M = robot->fMe(q);  // matrix form
        p.buildFrom(M);     // translation + angle-axis form

        if(robot->mode() == ecn::Robot::MODE_POSITION_MANUAL)
        {
            // just check the Direct Geometric Model
            // DONE: fill the fMw function
            robot->checkPose(M);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_MANUAL)
        {
            // follow a given operational velocity
            v = robot->vw();

            // DONE: fill the fJw function
            // DONE: compute vCommand
            vpMatrix Raux(6,6);
            vpRotationMatrix R = M.getRotationMatrix();


            ecn::putAt(Raux, R, 0, 0);
            ecn::putAt(Raux, R, 3, 3);

            vpColVector fVe(6);
            fVe = Raux * v;

            vCommand = robot->fJe(q).pseudoInverse() * fVe;

            robot->setJointVelocity(vCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_DIRECT_P2P)
        {
            // find the Inverse Geometry to reach Md
            // DONE: fill the inverseGeometry function
            qf = robot->inverseGeometry(Md, q);
            robot->setJointPosition(qf);
        }




        else if(robot->mode() == ecn::Robot::MODE_INTERP_P2P)
        {
            // reach Md with interpolated joint trajectory
            // use q0 (initial position), qf (final), aMax and vMax

            // if reference has changed, compute new tf
            if(robot->newRef())
            {
                q0 = robot->inverseGeometry(M0, q);
                qf = robot->inverseGeometry(Md, q);
                tf = 0;

                // iterating through all the joints to find the max tf and use it to sync all the other
                for (unsigned int i = 0; i < n; i++)
                {
                    auto dq = qf[i] - q0[i];
                    auto tfv = 3 * std::abs(dq) / (2 * vMax[i]);
                    auto tfa = sqrt(6 * std::abs(dq) / aMax[i]);
                    auto tfmax{0};

                    if (tfv > tfa)
                    {
                        tfmax = tfv;
                    }
                    else
                    {
                        tfmax = tfa;
                    }
                    if (tfmax > tf)
                    {
                        tf = tfmax;
                    }
                }
            }

            // DONE: compute qCommand from q0, qf, t, t0 and tf
            auto dq = qf - q0;      auto dt = t - t0;
            auto Pt = 3 * pow(dt / tf, 2) - 2 * pow(dt / tf, 3);
            qCommand = q0 + Pt * (dq);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_STRAIGHT_LINE_P2P)
        {
            // go from M0 to Md in 1 sec
            tf = 1;
            auto dt = t - t0;       auto alpha{1 * (dt > tf)};
            if (alpha == 0)
            {
                alpha = dt / tf;
            }

            // DONE: compute qCommand from M0, Md, t, t0 and tf
            // use robot->intermediaryPose to build poses between M0 and Md

            auto pose = robot->intermediaryPose(M0, Md, alpha);
            qCommand = robot->inverseGeometry(pose, q);

            robot->setJointPosition(qCommand);
        }


        else if(robot->mode() == ecn::Robot::MODE_VELOCITY_P2P)
        {
            // go to Md using operational velocity
            // Me is the Homo Matrix of final frame wrt intermediate desired frame
            auto Me = Md.inverse() * M;
            pe.buildFrom(Me);  // contains the pose data in t & thetau form
            auto eThetaU = pe.getThetaUVector();        auto eT = pe.getTranslationVector();
            auto lambda = - robot->lambda();          auto fRe = Md.getRotationMatrix();
            auto v = lambda * fRe * eT;
            auto w = lambda * fRe * eThetaU.getU() * eThetaU.getTheta();
            std::cout << "v : " << v << std::endl;
            std::cout << "w : " << w << std::endl;

            // DONE: compute joint velocity command

            for (int i = 0; i < 6; i++)
            {
                if (i < 3)
                {
                    ve[i] = v[i];
                }
                else
                {
                    ve[i] = w[i-3];
                }
            }

            vCommand = robot->fJe(q).pseudoInverse() * ve;
            robot->setJointVelocity(vCommand);
        }


    }
}
