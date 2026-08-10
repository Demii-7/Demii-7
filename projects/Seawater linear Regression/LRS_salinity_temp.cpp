//
//  LRS_salinity_temp.cpp
//  Freezing Temperature of Seawater
//
//  Created by Demarce Williams on 04/10/2023.

//  Description: The program computes the linear regression model based on a given data set of temperature and salinity readings and estimate the freezing temperatures based on a given salinity value of water using the computed model.
/*___________________________________________________________________________________________________________________________*/

#include <iostream> //Standard input/output library
#include <fstream> //Allows for file manipulation
#include <string> //Allows for string data manipulation
#include <cmath> //Provides math functions
#include <limits> //Allow to obtain properties of numeric data types

using namespace std;

int main() {
    
    //Object declaration
    // x and y represents the salinity and frezing temperature, respectively
    double x(0), y(0);    
    
    //Stores the summations of the respective parameters
    double sum_x(0),sum_y(0),prod_xy(0), square_x(0);     
    
    //Stores the calculated model parameters m and b in the regression model T = m * S + b
    double m(0), b(0);    
    
    //Counter to store total iterations and choice to store user selection from menu
    int ctr(0), choice(0);
    
    //n stores the total number of data sets and indicator tells whether a regression model is present
    int indicator(0);  
    
    //Stores file name with extension
    string file("");
    
    //Checks validity of input
    bool isValidInput = false;
    
    //Main menu loop
    do{
        // Program title and Welcome message
        cout << "\nFreezing Temperature of Seawater\n";
        cout << "\nWelcome to the Linear Regression Model System\n";
        
        // Prints menu to user
        cout << "\n1  : Calculate Linear Regression Model.\n";
        cout << "2  : Calculate Freezing Temperature.\n";
        cout << "0  : Exit Program.\n";
        
        //Ensures user input is of integer type
        do{
            // Prompts the user to make a selection from menu
            cout << "\nPlease choose an option from the menu above: ";
            cin >> choice;
              
            //Checks Validity of input
            if(cin.fail()){
                
                isValidInput = false;
                cout << "\nError! Please enter an integer!\n";
                
                //clears characters in input buffer up to the limit or upon encountering a newline character.
                cin.clear();
                cin.ignore(numeric_limits<streamsize>::max(),'\n');
            }else{
                isValidInput = true;
            }//End of if statement
            
        }while(!isValidInput);
        

        switch(choice){
            case 0:
                cout << "\nThank you for using our system...\n";
                cout << "Goodbye!\n" << endl;
                break;
            case 1:{
                // Prompts user to enter file name with extension and stores user input in string variable file.
                cout << "\nEnter the data file name with extension: ";
                cin >> file;
                
                // Open input file stream for given file.
                ifstream inFile;
                inFile.open(file);
                
                //Checks if file was succesfully opened.
                if(inFile.fail()){
                    cout << "\nERROR! Could not open file!\n" << endl;
                    exit(1);
                }//End of if statement
                

                //Loops through the file until end of the file is reached
                while(!inFile.eof()){
                    
                    // Reads values of salinity and temperature from file into x and y, respectively
                    inFile >> x >> y;
                    
                    //Skip header and any non-numeric character
                    if(inFile.fail()){
                        
                        //clears characters in input buffer up to the limit or upon encountering a newline character.
                        inFile.clear();
                        inFile.ignore(numeric_limits<streamsize>::max(),'\n');
                        continue;
                    }//End of if statement
                    
                     
                    //Calculate the summations of the respective parameters using salinity and temperature data from file
                    sum_x += x;
                    sum_y += y;
                    square_x += pow(x,2);
                    prod_xy += x * y;
                    
                    //Increment counter by 1
                    ctr++;

                }//End of while loop
                                
                // Calculate Linear Regression model parameters
                m = ((sum_x * sum_y) - (ctr * prod_xy)) / (pow(sum_x, 2) - (ctr * square_x));
                b = ((sum_x * prod_xy) - (square_x * sum_y)) / (pow(sum_x, 2) - (ctr * square_x));
                
                //Indicates to program that regression model was calculated
                indicator = 1;
                
                //Output Linear Regression model to user
                cout << "\nThe linear regression model is: \n" << "T = " << m << " * S + " << b << endl;
                break;
            }case 2:{
                //Check if parameter m or the slope is given to ensure that regression model is already
                if (indicator == 1){
                    
                    //Ensure salinity is a non-negative integer and not a character
                    do{
                        //Prompt suer to enter salinity of water
                        cout << "\nEnter the salinity of the water: ";
                        cin >> x;
                        
                        if(x < 0 || cin.fail()){
                            
                            isValidInput = false;
                            cout << "\nError! Please enter a non-negative inetger!\n";
                            
                            //clears characters in input buffer up to the limit or upon encountering a newline character.
                            cin.clear();
                            cin.ignore(numeric_limits<streamsize>::max(),'\n');
                        }else{
                            isValidInput = true;
                        }//End of if statement
                        
                    }while(!isValidInput);

                    //Calculate the freezing temperature of the weter.
                    y = (m * x) + b;
                    
                    //Output freezing temperature to the user
                    cout << "\nThe freezing temperature of water with salinity " << x << " ppt is: " << y << " F" << endl;
                }else{
                    cout << "\nERROR! Please calculate Linear Regression Model first!\n";
                    continue;
                }//End of if statement
                
                break;
            }default:{
                cout << "\n\nError! Please choose an option in the menu!!\n\n";
                continue;
            }//End of case 2 block
        }//End of switch statement
        
    }while(choice != 0); // End of main menu loop
    
    //Exit Program
    return 0;
}//Main function
