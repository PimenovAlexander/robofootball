//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
  \file    main.cpp
  \brief   The client application entry point.
  \author  Shirokolobov Ilya, (C) 2013
  \group   Robofootball's group, Russia
*/
//========================================================================

#include <stdio.h>
#include "RfClient.h"

int initConfig(RCConfig *config){
    ifstream configFile;
    string line;

    configFile.open("rf-client.cnf", ios::in);

    if(!configFile.is_open()){
        cerr << "Can't open configuration file" << endl;
        return -1;
    }

    if(!config){
            cerr << "Can't parse configuration file" << endl;
            return -2;
        }

    while(!configFile.eof()){
        getline(configFile, line);

        if(line.empty()){
            continue;
        }
        if(line.find("rfclient.name") != string::npos){
            config->name = line.substr(line.find('=') + 1);

            if(config->name.size() > 16){
                cerr << "[WARNING] rfclient.name value is too long." << endl << "It will be truncated up to 16 characters." << endl;
                config->name.resize(16);
            }
        }

        if(line.find("rfclient.file_of_matlab") != string::npos){
            string fom = line.substr(line.find('=') + 1);

            if(fom.size() > 16){
                cerr << "[WARNING] rfclient.file_of_matlab value is too long." << endl << "It will be truncated up to 16 characters." << endl;
                fom.resize(16);
            }
            char *str = new char[ fom.length() + 1 ];
            strcpy(str,  fom.c_str());
            config->file_of_matlab = str;
        }

        if(line.find("rfclient.RULE_AMOUNT") != string::npos){
            config->RULE_AMOUNT = atoi(line.substr(line.find('=') + 1).c_str());
        }

        if(line.find("rfclient.RULE_LENGTH") != string::npos){
            config->RULE_LENGTH = atoi(line.substr(line.find('=') + 1).c_str());
        }

        if(line.find("rfclient.BACK_AMOUNT") != string::npos){
            config->BACK_AMOUNT = atoi(line.substr(line.find('=') + 1).c_str());
        }

        if(line.find("rfclient.BACK_LENGTH") != string::npos){
            config->BACK_LENGTH = atoi(line.substr(line.find('=') + 1).c_str());
        }
    }

    configFile.close();

    return 0;
}

int main(int argc, char *argv[])
{
//    (void)argc;
//    (void)argv;

    QApplication app(argc, argv);

    RCConfig* rcconfig = new RCConfig();

    cout << "Initialization config file..." << endl;

    if(!initConfig(rcconfig)){

//        cout << rcconfig.file_of_matlab << endl;
//        cout << rcconfig.name << endl;
//        cout << rcconfig.BACK_AMOUNT << endl;
//        cout << rcconfig.BACK_LENGTH << endl;
//        cout << rcconfig.RULE_AMOUNT << endl;
//        cout << rcconfig.RULE_LENGTH << endl;

        cout << "...successful" << endl;
    }
    else
    {
        cerr << "...bad" << endl;
        char *str = new char[16];
        str = "Robofootball";
        rcconfig->name = str;
        str = "main";
        rcconfig->file_of_matlab=str;
        rcconfig->RULE_AMOUNT=5;
        rcconfig->RULE_LENGTH=5;
        rcconfig->BACK_AMOUNT=10;
        rcconfig->BACK_LENGTH=8;

    }

    RfData* data = new RfData(rcconfig);

    RfThread* thread = new RfThread(data);
    thread->run_matlab();

    RfClient rfclient(thread, data);
    thread->go();
//    rfclient.setGeometry(100, 100, 500, 355);
    rfclient.show();
    return app.exec();


  //  return 0;
}
