#include <iostream>
#include <ilcplex/ilocplex.h>
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;
ILOSTLBEGIN;

vector<vector<int>> findSubTours(vector<int> nextNode) {
    int n = nextNode.size();
    vector<int> visited(n, 0);
    vector<vector<int>> subTours;
    for(int i=0; i<n; i++) {
        if(visited[i])
            continue;
        int currentNode = i;
        vector<int> currentTour;
        do {
            currentTour.push_back(currentNode);
            visited[currentNode] = 1;
            currentNode = nextNode[currentNode];
        } while(currentNode != i);
        subTours.push_back(currentTour);
    }
    return subTours;
}

int main()
{
    freopen("data.txt", "rt", stdin);
    freopen("solution.txt", "wt", stdout);
    ios::sync_with_stdio(false);
    cin.tie(0), cout.tie(0);

    int n;
    cin>>n;

    vector<double> xPos(n), yPos(n);
    for(int i=0; i<n; i++) {
        cin>>xPos[i]>>yPos[i];
    }

    vector<vector<double>> cost(n, vector<double>(n));
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            cost[i][j] = sqrt(pow(xPos[j]-xPos[i], 2) + pow(yPos[j]-yPos[i], 2));
        }
    }


    IloEnv env;
    try{
        IloModel model(env);


        /*------DECISION VARIABLE-------*/
        // x[i][j] = 1 if place j is visited immediately after i, 0 otherwise
        IloArray<IloNumVarArray> x(env, n);
        for(int i=0; i<n; i++) {
            x[i] = IloNumVarArray(env, n);
            for(int j=0; j<n; j++) {
                x[i][j] = IloNumVar(env, 0, 1, ILOBOOL);
            }
        }


        /*------OBJECTIVE FUNTION--------*/
        IloExpr sumCost(env);
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                if(i!=j) {
                    sumCost += x[i][j]*cost[i][j];
                }
            }
        }
        model.add(IloMinimize(env, sumCost));
        sumCost.end();


        /*------CONSTRAINTS----------*/
        IloExpr expr(env);
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                if(i!=j) {
                    expr += x[i][j];
                }
            }
            model.add(expr == 1);
            expr.clear();
        }

        for(int j=0; j<n; j++) {
            for(int i=0; i<n; i++) {
                if(i!=j) {
                    expr += x[i][j];
                }
            }
            model.add(expr == 1);
            expr.clear();
        }

        // Third constraint, we make sure that no subset of n points forms a subtour
        /* But there are too many subsets (~2^n) so we will add the constraints one by one. This is like
        making the model tight gradually, until it is tight enough to give feasible results then stop */

        IloCplex cplex(model);
        while(1) {
            if(!cplex.solve()) {
                cout<< "Failed to optimize model with this data file!";
                break;
            }
            vector<int> nextNode(n, -1);
            for(int i=0; i<n; i++) {
                for(int j=0; j<n; j++) {
                    if(i!=j && cplex.getValue(x[i][j]) > 0.9) {
                        nextNode[i] = j;
                        break;
                    }
                }
            }
            vector<vector<int>> subTours = findSubTours(nextNode);
            if(subTours.size() == 1)
                break;
            int numOfSubTour = subTours.size();
            for(int i=0; i<numOfSubTour; i++) {
                int numOfNodes = subTours[i].size();
                for(int j=0; j<numOfNodes; j++) {
                    for(int k=0; k<numOfNodes; k++) {
                        if(subTours[i][j] != subTours[i][k]) {
                            expr += x[subTours[i][j]][subTours[i][k]];
                        }
                    }
                }
                model.add(expr <= numOfNodes - 1);
                expr.clear();
            }
        }
        expr.end();

        cplex.exportModel("TSPmodel_2.lp");


        /*-----PRINT SOLUTIONS-----------*/
        cout<< "The minimum cost = "<< cplex.getObjValue()<< "\n";
        cout<< "The path is:"<< "\n";
        // Suppose we start from place 2
        int start_vertex = 2;
        cout<< "2";
        int i=2;
        do {
            for(int j=0; j<n; j++) {
                if(i != j && cplex.getValue(x[i][j]) > 0.9) {
                    cout<< "->"<< j;
                    i = j;
                    break;
                }
            }
        } while(i != start_vertex);
    }

    catch (IloException& e) {
        cerr << "Conver exception caught: " << e << endl; // No solution exists
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

    env.end();
    return 0;
}
