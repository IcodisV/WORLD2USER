using System;
using MathNet.Numerics.LinearAlgebra;

namespace WORLD2USER
{
    class Program
    {
        static void Main(string[] args)
        {
            // Define user frame at (0, 0, 0)
            Matrix<double> userFrame = Matrix<double>.Build.DenseOfArray(new double[,] {
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            });

            // Take input of xyzwpr coordinates of user frame relative location to robot
            Console.WriteLine("Enter XYZWPR coordinates of user frame relative location to robot, separated by commas:");
            string[] inputStrs = Console.ReadLine().Split(',');
            double[] input = new double[inputStrs.Length];
            for (int i = 0; i < inputStrs.Length; i++)
            {
                input[i] = double.Parse(inputStrs[i]);
            }
            input[3] = Math.PI * input[3] / 180; // convert W to radians
            input[4] = Math.PI * input[4] / 180; // convert P to radians
            input[5] = Math.PI * input[5] / 180; // convert R to radians

            // Convert xyzwpr to matrix
            Matrix<double> userFrameMatrix = ConvertXYZWPRtoMatrix(input);

            // Calculate transformation from user frame to robot frame
            Matrix<double> robotToUserFrame = userFrameMatrix.Inverse() * userFrame;

            // Take input of a point on the user frame
            Console.WriteLine("Enter XYZ coordinates of a point on the user frame, separated by commas:");
            inputStrs = Console.ReadLine().Split(',');
            double[] pointInput = new double[inputStrs.Length];
            for (int i = 0; i < inputStrs.Length; i++)
            {
                pointInput[i] = double.Parse(inputStrs[i]);
            }
            pointInput = new double[] { pointInput[0], pointInput[1], pointInput[2], 1 }; // add homogeneous coordinate

            Vector<double> userPoint = Vector<double>.Build.DenseOfArray(pointInput);

            // Calculate location of user point in robot frame
            Vector<double> robotPoint = robotToUserFrame * userPoint;

            // Calculate location of user frame origin in robot frame
            Vector<double> robotUserFrameOrigin = robotToUserFrame * userFrame.Column(3);

            // Extract x, y, z, w, p, r from robot frame to user frame transformation
            double x = robotToUserFrame[0, 3];
            double y = robotToUserFrame[1, 3];
            double z = robotToUserFrame[2, 3];
            double w = Math.Atan2(robotToUserFrame[2, 1], robotToUserFrame[2, 2]);
            double p = Math.Asin(-robotToUserFrame[2, 0]);
            double r = Math.Atan2(robotToUserFrame[1, 0], robotToUserFrame[0, 0]);

            // Print results
            Console.WriteLine($"User frame matrix:\n{userFrameMatrix}");
            Console.WriteLine($"Robot to user frame transformation:\n{robotToUserFrame}");
            Console.WriteLine($"User point in robot frame:\n{robotPoint}");
            Console.WriteLine($"User frame origin in robot frame:\n{robotUserFrameOrigin}");
            Console.WriteLine($"x={x}, y={y}, z={z}, w={w}, p={p}, r={r}");
            Console.ReadLine();
        }

        public static Matrix<double> ConvertXYZWPRtoMatrix(double[] input)
        {
            Matrix<double> resultMatrix = Matrix<double>.Build.DenseIdentity(4, 4);
            Matrix<double> rX = Matrix<double>.Build.DenseIdentity(4, 4);
            Matrix<double> rY = Matrix<double>.Build.DenseIdentity(4, 4);
            Matrix<double> rZ = Matrix<double>.Build.DenseIdentity(4, 4);
            double x = input[0];
            double y = input[1];
            double z = input[2];
            double w = input[3];
            double p = input[4];
            double r = input[5];

            // R
            rZ[0, 0] = Math.Cos(r);
            rZ[0, 1] = -Math.Sin(r);
            rZ[1, 0] = Math.Sin(r);
            rZ[1, 1] = Math.Cos(r);

            rX[1, 1] = Math.Cos(p);
            rX[1, 2] = -Math.Sin(p);
            rX[2, 1] = Math.Sin(p);
            rX[2, 2] = Math.Cos(p);

            rY[0, 0] = Math.Cos(w);
            rY[0, 2] = Math.Sin(w);
            rY[2, 0] = -Math.Sin(w);
            rY[2, 2] = Math.Cos(w);

            // Translation
            resultMatrix[0, 3] = x;
            resultMatrix[1, 3] = y;
            resultMatrix[2, 3] = z;

            // Multiply in reverse order to obtain correct transform
            resultMatrix = rX * rY * rZ * resultMatrix;

            return resultMatrix;
        }
    }
}
