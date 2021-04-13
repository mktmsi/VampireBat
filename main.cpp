//毎度ファイル保存するのではなく，配列に格納しておいて一定期間ごとにまとめてファイル出力を行う．
//シェアリング時の好感度変動
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <GLUT/glut.h>
#include <time.h>
#include "Monitor.hpp"
//#include <opencv/cv.h>
#include "Vector.h"
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <png.h>
#define StrNum (30)
#define dt (0.01) //タイムステップ
//#define PI (3.14159265359)
#define N (15) //質点数
#define zoom (20)
#define FoodNum (7)
#define GridNum (200)
Monitor monitor;

#define F(i, j) F[N * (i) + (j)]
#define AgentsInsideAgentRadius(i, j) AgentsInsideAgentRadius[N * (i) + (j)]
#define AgentInsideSensorRange(i, j) AgentInsideSensorRange[N * (i) + (j)]

Vector2D r_old[N]; //質点の位置ベクトル
Vector2D r_new[N]; //質点の位置ベクトル
Vector2D v_old[N];
Vector2D v_new[N];
Vector2D rw[N]; //質点の位置ベクトル
Vector2D rij[N][N];
Vector2D rijHat[N][N];

double FSum[N] = {};
int RWInterval[N];                   //ランダムウォーク方向転換の間隔
double F[N * N] = {0.0};             //好感度
double S_old[N] = {}, S_new[N] = {}; //体力
int GroomFlag[N] = {}, SharingFlag[N] = {};
int Dead[N] = {0};
int NumOfDeadCell = 0;
int NumOfSharing = 0, NumOfCollision = 0;
int NumOfGoSharing = 0, TotalSharing = 0;

FILE *fpTotalSharing;
FILE *fpWhole, *fpFijDatas, *fpAgentDatas, *fpSharePartners; //, *fpEach; //fpというファイル用変数を定義
int FileCount = 0;
char strFolderParameterSetCount[100] = "", strFolderWholes[200] = "", strFolderFijDatas[200] = "", strWhole[200] = "", strFijDatas[200] = "", strAgentDatas[200] = "", strSharePartners[200] = ""; //, strEach[50];
char strFolder3[200] = "", strFolderAgentDatas[200] = "", strFolderSharePartner[80] = "", strParaSetTxt[100] = "";
char Whole[StrNum][20]; //[文字列数][文字列の長さ]
char strTotalSharing[200] = "";
//char Each[2 * StrNum][400];
int DataCount = 0;

long long int step = 0; //現在のタイムステップ
double t = 0.0000;

const int RWIntervalMax = 500.0, RWIntervalMin = 50.0; //ランダムウォークのインターバル
const double SensorRange = 10.50, PrivateZone = 3.0;   //SensorRange = 8.50, PrivateZone = 3.0;
const double L = 30.0;                                 //境界の範囲

//移動の方程式のパラメータ
const double beta = 0.900, m = 5.0;
double kr = 0.5010, ka = 0.50510; //
double kg = 1.61010, kj = 1.5;    //kg = 0.61010, kj = 0.5;
const double vmax = 3.0;
const double mu = 0.60, k = 5.070; //mu=0.8以上にするとぶっ飛ぶ

//エサ
int InitFoodNum = 5;
double Food0 = 1.0, time_th = 0.3; //time_th = 0.3;
double foodx[N] = {}, foody[N] = {};
int InFood[N] = {};
int NumOfGrids = GridNum / 2 * (SensorRange / L);
double FoodAbsr = 0.0, wide = 2.0, pow_absr[N] = {};

//体力
double cs = 0.0001, cv = 0.0001; //cs = 0.0001, cv = 0.0001;
const double c_eat = 0.50;       //c_eat=1.0;1ステップあたりに食べるエサ量
const double Slimit = 1.0;       //Hlimit=1000.0,Hmax,Hmin;//ランダムウォークつき非対称相互作用
double Smax = 0.0, Smin = 0.0;

//double food_init = 500.0 * 0.001, foodAreaInit = 1.0;
//int FoodChangeInterval = 30000;
//好感度・毛繕い
int NumOfAgentInsideAgentRadius[N] = {}, NumOfAgentInsideSensorRange[N] = {};
int AgentsInsideAgentRadius[N * N] = {}, AgentInsideSensorRange[N * N] = {};
double Finit = 0.0, Fmax = 0.0, Fmin = 0.0, Flimit = 1.0;
double cgroom = 0.001; //毛繕いによる好感度の増加
double DistanceBetweenIJ[N][N] = {};
//food sharing
double k2 = 10.0, eps1 = 0.0001, ks = 0.00; // k2 = 1.10, k3 = 1.0, k4 = 1.0, eps2 = 0.1, eps1 = 0.000001;
const double cShare = 0.2, fshare = 0.01;
int SharingGoTo[N] = {};
double AgentRadius = 1.00;
double Fsum[N] = {}, FThreshHigh = 0.8;
int Donating[N] = {}, Recieving[N] = {};

int DisplayInterval = 500;
int FlagRandomWalkOnly = 0; //他個体との非対称相互作用
//int FlagAltruisticOn = 1, FlagGrooming = 1, FlagFoodSharing = 1; //毛づくろいとシェアリング
int FlagRandomWalk = 1, FlagDataSave = 1; //0:off a:on
int SaveDataToFilesIntervals = 100;
const int save_flag = 0; //0:画像保存しない，1:画像保存する
int FlagFood = 1;
int ToEndFileCount = 100; //200;
int winid;
int ParameterSetCount = 0, EndSetCount = 11;
int count1 = 0, count2 = 0;
double EndTime = 500.0; //not timestep
int Flaaaag = 0;

struct esa
{
  double amount;
  double AmountInit;
  double radius;
};

void capture(long long int);

struct esa VV[GridNum][GridNum];

void CheckDistance()
{
  int j = 0, i = 0;

  for (i = 0; i < N; i++)
  {
    NumOfAgentInsideAgentRadius[i] = 0, NumOfAgentInsideSensorRange[i] = 0;
    AgentsInsideAgentRadius(i, 0) = i, AgentInsideSensorRange(i, 0) = i;
  }
  for (i = 0; i < N; i++)
  {
    for (j = 0; j < N; j++)
    {
      if (i != j && S_old[i] > 0.0 && S_old[j] > 0.0) //調べる相手を選択
      {
        rij[i][j] = r_old[j] - r_old[i];
        DistanceBetweenIJ[i][j] = rij[i][j].get_abs();
        //printf("distance=%f\n",DistanceBetweenIJ[*pi][j]);
        if (DistanceBetweenIJ[i][j] == 0.0)
        { //吹っ飛び防止に非ゼロの距離を代入
          printf("checkd\n");
          rij[i][j].set_vec(rand() / (1.0 + RAND_MAX), rand() / (1.0 + RAND_MAX));
          DistanceBetweenIJ[i][j] = rij[i][j].get_abs();
        }

        rijHat[i][j] = rij[i][j] / DistanceBetweenIJ[i][j]; //両者の距離を正規化
        //
        if (DistanceBetweenIJ[i][j] > SensorRange) //個体jが個体iのセンサレンジに入っていなければ
        {
          //何もしない
        }
        else //個体jが個体iのセンサレンジに入っていれば
        {
          AgentInsideSensorRange(i, NumOfAgentInsideSensorRange[i]) = j;
          NumOfAgentInsideSensorRange[i]++;
          if (DistanceBetweenIJ[i][j] <= 2.0 * AgentRadius) //個体jが個体iに接して入れば
          {
            AgentsInsideAgentRadius(i, NumOfAgentInsideAgentRadius[i]) = j;
            NumOfAgentInsideAgentRadius[i]++;
          }
        }
      }
    }
  }
}

void ParameterInitialization()
{
  char tmp[256];
  char *p, *p2, *r;
  char **endptr = NULL;
  int fileNum = 0, i = 0;
  char strPos[128];

  //現在のディレクトリを取得
  p = getcwd(tmp, sizeof(tmp) - 1);
  printf("tmp=%s\n", tmp);
  strPos[0] = '/';
  strcpy(strPos + 1, tmp);

  //フォルダの番号を探す
  char str[] = "";
  int sizeNum = 0, sizeCharas = 0;
  r = strtok(strPos, "/"); //一文字目を切り出します。
  sizeCharas = strlen(p);
  strcat(str, "/");
  strcat(str, r);
  while (r = strtok(NULL, "/"))
  { //次の文字を切り出します。無ければNULLを返します。
    fileNum = (int)(strtol(r, endptr, 10));
    sizeNum = strlen(r);
    printf("%s\n", r);
    strcat(str, "/");
    strcat(str, r);
  }
  printf("fileNum=%d\n", fileNum);
  printf("%s\n", str);
  printf("sizeCharas=%d\n", sizeCharas);
  printf("sizeNum=%d\n", sizeNum);

  //parameters.csvのディレクトリを抜き出す
  char strDir[sizeCharas - sizeNum + 1];
  strncpy(strDir, tmp, sizeCharas - sizeNum);
  sizeCharas - sizeNum + 1;
  strDir[sizeCharas - sizeNum] = '\0';
  printf("strDir=%s\n", strDir);

  //parameterset.csvをオープンする
  strcat(strDir, "/parameters.csv");
  char *fname = strDir; //"/Users/mikamihiroshi/Document/Research/programs/check/parameters.csv";
  FILE *fp;
  fp = fopen(fname, "r");
  if (fp == NULL)
  {
    printf("%sファイルが開けません\n", fname);
    exit(0);
  }
  printf("aaaa\n");
  //csvファイルからパラメータ値を代入する
  char temp;
  char StrCs1[200], StrCs2[200], StrCs3[200], StrCs4[200], StrCs5[200], StrCs6[200], StrCs7[200], StrCs8[200], StrCs9[200], StrCv[200];
  fscanf(fp, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%s", StrCs1, StrCs2, StrCs3, StrCs4, StrCs5, StrCs6, StrCs7, StrCs8, StrCs9, StrCv);
  printf("bbbb\n");
  for (i = 0; i < fileNum; i++)
  { //csvファイルの1行目の数値はなぜか読み取ってくれない
    fscanf(fp, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%d,%lf", &ParameterSetCount, &EndSetCount, &eps1, &cgroom, &cs, &cv, &time_th, &kg, &ToEndFileCount, &ka);
  }
  printf("ParaSet=%d EndSet=%d eps1=%lf cgroom=%lf cs=%lf cv=%lf time_th=%lf kr=%lf ka=%lf kg=%lf \n", ParameterSetCount, EndSetCount, eps1, cgroom, cs, cv, time_th, kr, ka, kg);
}

void init()
{
  int ransu_x = 0, ransu_y = 0;
  int ParaCountTemp = 0;
  int i = 0, j = 0;
  char filename[256], str[20];
  char tmp[256]; //tmp1[256] = "";
  char *p;

  if (Flaaaag == 0)
  { //シミュレーション開始時の処理
    ParameterInitialization();
    Flaaaag = 1;
    count1 = ParameterSetCount % 11;
    count2 = ParameterSetCount / 11;
    printf("count1=%d count2=%d\n", count1, count2);
  }
  if (FlagDataSave == 1 && FileCount == ToEndFileCount)
  { //パラメータの更新
    FileCount = 0;
    ParameterSetCount++;
    if (count1 % 10 == 0 && count1 != 0)
    {
      count1 = 0;
      count2++;
    }
    else
    {
      count1++;
    }
    printf("ParameterSetCount=%d", ParameterSetCount);
  }
  k2 = (double)(count1)*1.0;
  ks = 0.001 * pow(10.0, (double)count2);
  NumOfDeadCell = 0;
  step = 0;
  t = 0.0;
  DataCount = 0;
  double Tem = 0.0;

  srand((unsigned int)time(NULL)); //乱数の初期化
  for (i = 0; i < 5; i++)
  {
    rand();
  }

  for (i = 0; i < N; i++)
  {
    SharingGoTo[i] = i;
    Fsum[i] = 0.0;
  }

  for (i = 0; i < N; i++)
  {
    Recieving[i] = 0, Donating[i] = 0; //シェアリングを受けた回数とやった回数（記録用）
    v_old[i].set_vec(0.0, 0.0);
    v_new[i] = v_old[i];
    RWInterval[i] = (int)(RWIntervalMin) + (int)(rand() * (RWIntervalMax - RWIntervalMin + 1.0)) / (1.0 + RAND_MAX);
    //Dead[i] = 0;
    //体力の初期化
    S_new[i] = Slimit;
    S_old[i] = S_new[i];
    //座標の初期化
    r_old[i].set_vec(-L + (rand() * 2.0 * L) / (1.0 + RAND_MAX), -L + (rand() * 2.0 * L) / (1.0 + RAND_MAX));
    r_new[i] = r_old[i];
    //r[i].set_vec(L / 2.0, L / 2.0);
    //ランダムウォーク初期化
    if (FlagRandomWalk)
    {
      double rwAbs = 0.0;
      do
      {
        rw[i].set_vec(-1.0 + (rand() * 2.0) / (1.0 + RAND_MAX), -1.0 + (rand() * 2.0) / (1.0 + RAND_MAX));
      } while (rwAbs = rw[i].get_abs() == 0.0);
      rw[i].set_vec(rw[i].get_x() / rwAbs, rw[i].get_y() / rwAbs);
      //printf("check\n");
    }
    for (j = 0; j < N; j++)
    {
      F(i, j) = Finit; //個体iからjに対する好感度の初期化
      if (i == j)
      {
        F(i, j) = 0.0;
      }
    }
  }

  //エサの初期化
  for (i = 0; i < GridNum; i++) //エサをフィールドからなくす
  {
    for (j = 0; j < GridNum; j++)
    {
      VV[i][j].amount = Tem;
      VV[i][j].AmountInit = Tem;
      VV[i][j].radius = wide * Tem / Food0;
    }
  }
  if (FlagFood) //エサをフィールドに出現させる時
  {
    do
    {
      ransu_x = (int)(rand() % GridNum), ransu_y = (int)(rand() % GridNum);
      if (VV[ransu_x][ransu_y].amount == 0.0)
      {
        VV[ransu_x][ransu_y].amount = Food0;
        VV[ransu_x][ransu_y].AmountInit = Food0;
        VV[ransu_x][ransu_y].radius = wide * Food0 / Food0;
        i++;
      }
    } while (i < InitFoodNum); //設定したエサの個数InitFoodNumになるまで繰り返す
  }
  if (FlagDataSave == 1) //データ保存
  {
    //kr = (double)(count1 + 1) * 0.40;
    //kg = (double)(count2 + 1) * 0.40;

    printf("k2=%.3f ks=%.3f\n", k2, ks);
    ////フォルダ＆ファイル作成/////
    p = getcwd(tmp, sizeof(tmp) - 1);
    if (p == NULL)
    {
      printf("ERROR,getcwd() ret=NULL\n");
      perror("getcwd");
      exit(EXIT_FAILURE);
    }

    printf("cwd = %s\n", tmp);
    if (FileCount == 0)
    {
      //Parametersetごとのフォルダを作成
      sprintf(strFolderParameterSetCount, "%s/Results/ParameterSet%d", tmp, ParameterSetCount);
      // printf("strFolderParameterSetCount = %s\n", strFolderParameterSetCount);
      if (mkdir(strFolderParameterSetCount, S_IRWXU) == 0)
      { //フォルダ作成
        printf("フォルダ作成に成功しました。\n");
      }
      else
      {
        printf("フォルダ作成に失敗しました。\n");
        printf("%s\n", strFolderParameterSetCount);
        exit(0);
      }
      //Parametersetごとのフォルダを作成
      sprintf(strFolderParameterSetCount, "%s/Results/ParameterSet%d", tmp, ParameterSetCount);
      sprintf(strParaSetTxt, "%s/ParaSet.txt", strFolderParameterSetCount);
      fpWhole = fopen(strParaSetTxt, "w");
      fprintf(fpWhole, "eps1=%lf cgroom=%lf,cs=%lf,cv=%lf,time_th=%lf,k2=%lf,ks=%lf,beta=%lf,m=%lf,kr=%lf,ka=%lf,kg=%lf,kj=%lf,mu=%lf,k=%lf,ceat=%lf,cshare=%lf,fshare=%lf,ToendFildCount=%d\n", eps1, cgroom, cs, cv, time_th, k2, ks, beta, m, kr, ka, kg, kj, mu, k, c_eat, cShare, fshare, ToEndFileCount);
      fclose(fpWhole);

      //残存個体数用のフォルダを作成
      sprintf(strFolderParameterSetCount, "%s/Results/ParameterSet%d", tmp, ParameterSetCount);
      sprintf(strFolderWholes, "%s/Wholes", strFolderParameterSetCount);
      if (mkdir(strFolderWholes, S_IRWXU) == 0)
      { //フォルダ作成
        printf("フォルダ作成に成功しました。\n");
      }
      else
      {
        printf("フォルダ作成に失敗しました。\n");
        printf("%s\n", strFolderWholes);
        exit(0);
      }
    }

    //////ファイルオープン///////
    //数値データ保存ファイルのファイル名設定
    sprintf(strWhole, "%s/Whole%d.csv", strFolderWholes, FileCount);
    printf("strWhole=%s\n", strWhole);
    fpWhole = fopen(strWhole, "a");
    fprintf(fpWhole, "tim,NumOfAgents,SumOfH,SumOfF,NumOfFHighs,NumOfFMids,NumOfLows,NumOfBelows\n");
    /////
    sprintf(strFolderParameterSetCount, "%s/Results/ParameterSet%d", tmp, ParameterSetCount);
    fclose(fpWhole);
    ///

    FileCount++;
  }
}

void CollisionWithOthers(int **ppi, Vector2D *pFphs)
{
  int j = 0;
  double temp = 0.0;
  (*pFphs).set_vec(0.0, 0.0);
  // NumOfCollision=0;
  for (j = 0; j < NumOfAgentInsideAgentRadius[**ppi]; j++) //個体1に接している個体について
  {
    NumOfCollision++;
    temp = 2.0 * AgentRadius - DistanceBetweenIJ[**ppi][AgentsInsideAgentRadius(**ppi, j)];
    if (temp == 0.0000) //両者が完全にかぶっちゃってたら
    {
      temp = 0.0010; //ぶっ飛び防止に少し距離があることにする
    }
    //printf("temp1=%f\n",temp);
    temp = pow(temp, -mu);
    if (temp < 0.0)
    {
      temp = 0.0;
    }
    //printf("temp2=%.3f\n",temp);
    (*pFphs) += (rijHat[**ppi][AgentsInsideAgentRadius(**ppi, j)] * (-k)) * temp;
    //printf("%f\n", (*pFphs).get_abs());
  }
  if (isnan((*pFphs).get_abs()))
  {
    int i = 0;
    printf("Fphs is nan\n");
    if (isnan(temp))
    {
      printf("temp is nan\n");
    }
    for (i = 0; i < N; i++)
    {
      printf("x=%.2f y=%.2f v=%.2f\n", r_old[i].get_x(), r_old[i].get_y(), v_old[i].get_abs());
    }
    exit(0);
  }
}

void CheckBorder(double *pCheck, double *pv_check)
{
  double absrrr = 0.0;
  //境界判定，速度転換
  if (*pCheck > L)
  {
    (*pCheck) = L;
    do
    {
      *pv_check = -1.0 * rand() / (1.0 + RAND_MAX);
    } while (*pv_check >= 0.0);
  }
  else if (*pCheck < -L)
  {
    //(*pr).set_vec(-L, (*pr).get_y());
    *pCheck = -L;
    do
    {
      *pv_check = 1.0 * rand() / (1.0 + RAND_MAX);
    } while (*pv_check <= 0.0);
  }
}

void CheckFoods(int **ppi, Vector2D *pgl)
{
  int k = 0, l = 0, w = 0;
  //double abs_min = L * 2.0 * L * 2.0;
  pow_absr[**ppi] = L * 2.0 * L * 2.0;
  int temp_x = 0, temp_y = 0;                                                                                                         //マス目番号!=座標
  int agent_i = GridNum / 2 + GridNum / 2 * r_old[**ppi].get_x() / L, agent_j = GridNum / 2 + GridNum / 2 * r_old[**ppi].get_y() / L; //個体が存在するグリッドを取得
  InFood[**ppi] = 0;
  for (w = 1; w <= NumOfGrids; w++) //最寄りのエサを探す
  {
    temp_x = agent_i - w; //個体の左側
    if (0 <= temp_x && temp_x < GridNum)
    {
      for (l = agent_j - w; l <= agent_j + w; l++)
      {
        if (0 <= l && l < GridNum)
        {
          if (VV[temp_x][l].amount > 0.0)
          {
            foodx[**ppi] = (double)temp_x;
            foody[**ppi] = (double)l;
            InFood[**ppi] = 1;
            pow_absr[**ppi] = ((temp_x - agent_i) * (temp_x - agent_i) + (l - agent_j) * (l - agent_j)) * 4.0 * L * L / (GridNum * GridNum);
            // goto loopend; //エサ探しループ抜ける
            break;
          }
        }
      }
    }

    if (InFood[**ppi] == 1) //エサが見つかって入れば
    {
      //何もしない
    }
    else //エサ探す
    {
      temp_x = agent_i + w; //個体の右側
      if (0 <= temp_x && temp_x < GridNum)
      {
        for (l = agent_j - w; l <= agent_j + w; l++)
        {
          if (0 <= l && l < GridNum)
          {
            if (VV[temp_x][l].amount > 0.0) //マス目番号(temp_x,l)にエサがあれば
            {
              foodx[**ppi] = (double)temp_x;
              foody[**ppi] = (double)l;
              InFood[**ppi] = 1;
              pow_absr[**ppi] = (temp_x - agent_i) * (temp_x - agent_i) + (l - agent_j) * (l - agent_j);
              // goto loopend; //エサ探しループ抜ける
              break;
            }
          }
        }
      }
    }

    if (InFood[**ppi] == 1)
    {
    }
    else
    {
      temp_y = agent_j - w;
      if (0 <= temp_y && temp_y < GridNum)
      {
        for (k = agent_i - w + 1; k <= agent_i + w - 1; k++)
        {
          if (k < GridNum && 0 <= k)
          {
            if (VV[k][temp_y].amount > 0.0)
            {
              foodx[**ppi] = (double)k;
              foody[**ppi] = (double)temp_y;
              InFood[**ppi] = 1;
              pow_absr[**ppi] = (k - agent_i) * (k - agent_i) + (temp_y - agent_j) * (temp_y - agent_j);
              //goto loopend; //エサ探しループ抜ける
              break;
            }
          }
        }
      }
    }
    if (InFood[**ppi] == 1)
    {
    }
    else
    {
      temp_y = agent_j + w;
      if (temp_y < GridNum && 0 <= temp_y)
      {
        for (k = agent_i - w + 1; k <= agent_i + w - 1; k++)
        {
          if (k < GridNum && 0 <= k)
          {
            if (VV[k][temp_y].amount > 0.0)
            {
              foodx[**ppi] = (double)k;
              foody[**ppi] = (double)temp_y;
              InFood[**ppi] = 1;
              pow_absr[**ppi] = (k - agent_i) * (k - agent_i) + (temp_y - agent_j) * (temp_y - agent_j);
              //goto loopend; //エサ探しループ抜ける
              break;
            }
          }
        }
      }
    }
  }

  double gl_abs = 0.0;
  if (InFood[**ppi] == 1)
  {
    //最寄りのエサへのベクトルを取得                                                                                                                              //エサが見つかっていれば実行
    (*pgl).set_vec((L * (foodx[**ppi] - (double)(GridNum / 2)) / (double)(GridNum / 2)) - r_old[**ppi].get_x(), (L * (foody[**ppi] - (double)(GridNum / 2)) / (double)(GridNum / 2)) - r_old[**ppi].get_y()); //この一行だけでめっちゃ重くなる
    gl_abs = (*pgl).get_abs();
    if (gl_abs == 0.0)
    {
      gl_abs = 0.001;
    }
    (*pgl).set_vec((*pgl).get_x() / gl_abs, (*pgl).get_y() / gl_abs);
  }
}

void InteractionWithOthers(Vector2D *SumOfInteraction, int **ppi)
{
  int j = 0, k = 0;

  (*SumOfInteraction).set_vec(0.0, 0.0);
  //他エージェントとの相互作用
  for (j = 0; j < NumOfAgentInsideSensorRange[**ppi]; j++)
  {
    k = AgentInsideSensorRange(**ppi, j); //センサレンジ内にいる個体番号を取得
    (*SumOfInteraction) += (rijHat[**ppi][k] * (F(**ppi, k) * ka / DistanceBetweenIJ[**ppi][k] - 1 / pow(DistanceBetweenIJ[**ppi][k], 2.0)));
  }
}

void UpdateRandomWaliDirection(int **ppi)
{
  double abs_v = 0.0;
  do
  {
    rw[**ppi].set_vec(-1.0 + (rand() * 2.0) / (1.0 + RAND_MAX), -1.0 + (rand() * 2.0) / (1.0 + RAND_MAX));
    abs_v = rw[**ppi].get_abs();
  } while (abs_v == 0.0);
  rw[**ppi].set_vec(rw[**ppi].get_x() / abs_v, rw[**ppi].get_y() / abs_v);
  RWInterval[**ppi] = RWIntervalMin + (int)(rand() * (RWIntervalMax - RWIntervalMin + 1.0) / (1.0 + RAND_MAX));
}

void SharingOff(int *pi)
{
  int j = 0, q = 0;
  double vmax = 3.0, abs_v = 0.0;
  Vector2D SumOfInteraction, Fphs;
  //微分方程式を解く，座標更新
  Fphs.set_vec(0.0, 0.0);
  SumOfInteraction.set_vec(0.0, 0.0);

  if (FlagRandomWalkOnly == 0 && AgentInsideSensorRange(*pi, 0) != (*pi))
  {
    //他個体との相互作用
    InteractionWithOthers(&SumOfInteraction, &pi);
  }
  //ランダムウォークの方向更新
  if (step % (unsigned long int)(RWInterval[*pi]) == 0 && FlagRandomWalk)
  {
    UpdateRandomWaliDirection(&pi);
  }
  //他者からの衝突反力
  if (AgentsInsideAgentRadius(*pi, 0) != (*pi))
  {
    CollisionWithOthers(&pi, &Fphs);
  }

  ///////////最近傍のエサからの引力///////////
  foodx[*pi] = 0.0;
  foody[*pi] = 0.0;
  InFood[*pi] = 0;
  int k = 0, l = 0, w = 0;
  //double abs_min = L * 2.0 * L * 2.0;
  pow_absr[*pi] = L * 2.0 * L * 2.0;
  int temp_x = 0, temp_y = 0;
  //double food_x=0.0,food_y=0.0;                                                                                  //センサレンジ内でエサがあるか探索
  Vector2D gl;
  gl.set_vec(0.0, 0.0);
  CheckFoods(&pi, &gl);
  ////////エサからの引力　終わり////////
  ///////////速度更新//////////////////
  double vv_abs = 0.0;
  v_new[*pi] = (SumOfInteraction - v_old[*pi] * beta + rw[*pi] * kr + gl * kg + Fphs) * dt / m + v_old[*pi];
  vv_abs = v_new[*pi].get_abs();

  if (vv_abs > vmax)
  {
    v_new[*pi].set_vec(v_new[*pi].get_x() * vmax / vv_abs, v_new[*pi].get_y() * vmax / vv_abs);
  }
  vv_abs = v_new[*pi].get_abs();
  if (isnan(vv_abs))
  {
    printf("v is nan1\n");
    exit(0);
  }

  //座標更新
  r_new[*pi] = r_old[*pi] + v_old[*pi] * dt;

  //境界判定，速度転換
  double check = 0.0;
  double v_check = 0.0;
  if (abs(r_new[*pi].get_x()) > L)
  {
    check = r_new[*pi].get_x();
    CheckBorder(&check, &v_check);
    r_new[*pi].set_x(check);
    rw[*pi].set_x(v_check);
  }
  if (abs(r_new[*pi].get_y()) > L)
  {
    check = r_new[*pi].get_y();
    CheckBorder(&check, &v_check);
    r_new[*pi].set_y(check);
    rw[*pi].set_y(v_check);
  }
  r_old[*pi] = r_new[*pi];
  v_old[*pi] = v_new[*pi];
}

void GoForSharing(int *pi, int *pj)
{ //シェアリング相手に向けて移動
  double vv_abs = 0.0;
  int k = 0;
  Vector2D Fphs;
  Fphs.set_vec(0.0, 0.0);
  foodx[*pi] = 0.0;
  foody[*pi] = 0.0;
  InFood[*pi] = 0;

  if (AgentsInsideAgentRadius(*pi, 0) != (*pi))
  {
    CollisionWithOthers(&pi, &Fphs);
  }

  v_new[*pi] = (v_old[*pi] * -1.0 * beta + Fphs + rijHat[*pi][*pj] * kj) * dt / m + v_old[*pi];
  vv_abs = v_new[*pi].get_abs();
  if (vv_abs > vmax)
  {
    v_new[*pi].set_vec(v_new[*pi].get_x() * vmax / vv_abs, v_new[*pi].get_y() * vmax / vv_abs);
  }
  vv_abs = v_new[*pi].get_abs();
  if (isnan(vv_abs))
  {
    printf("v is nan 2\n");
    exit(0);
  }
  //座標更新
  r_new[*pi] = r_old[*pi] + v_old[*pi] * dt;

  //境界判定，速度転換
  double check = 0.0;
  double v_check = 0.0;
  if (abs(r_new[*pi].get_x()) > L)
  {
    check = r_new[*pi].get_x();
    CheckBorder(&check, &v_check);
    r_new[*pi].set_x(check);
    rw[*pi].set_x(v_check);
  }
  if (abs(r_new[*pi].get_y()) > L)
  {
    check = r_new[*pi].get_y();
    CheckBorder(&check, &v_check);
    r_new[*pi].set_y(check);
    rw[*pi].set_y(v_check);
  }
  r_old[*pi] = r_new[*pi];
  v_old[*pi] = v_new[*pi];
}

void DoSharing()
{
  int k = 0, i = 0, j = 0;
  double dSij = 0.0;
  char str[20];
  for (i = 0; i < N; i++)
  {
    if (SharingGoTo[i] == i)
    {
    }
    else
    { //シェアのパートナーが決まっている場合
      rij[i][SharingGoTo[i]] = r_new[SharingGoTo[i]] - r_new[i];
      DistanceBetweenIJ[i][SharingGoTo[i]] = rij[i][SharingGoTo[i]].get_abs();
      if (DistanceBetweenIJ[i][SharingGoTo[i]] <= 2.0 * AgentRadius)
      { //両者が接して入れば
        dSij = S_old[i] - S_old[SharingGoTo[i]];
        if (abs(dSij) < cShare) //体力差が基準以下なら
        {
          //シェアしない
          SharingGoTo[SharingGoTo[i]] = SharingGoTo[i];
          SharingGoTo[i] = i; //シェアリング相手の登録解除
        }
        else
        {
          if (dSij > 0)
          { //自分の方が体力があれば
            NumOfSharing++;
            Donating[i]++;
            Recieving[SharingGoTo[i]]++;
            //体力の更新
            S_new[i] -= cShare;
            S_new[SharingGoTo[i]] += cShare; //AgentsInsideAgentRadiusの初期化はCheckDistance関数内で
            F(SharingGoTo[i], i) += fshare;
            F(i, SharingGoTo[i]) += fshare;
            if (F(SharingGoTo[i], i) >= Flimit)
            {
              F(SharingGoTo[i], i) = Flimit;
            }
            if (F(i, SharingGoTo[i]) >= Flimit)
            {
              F(i, SharingGoTo[i]) = Flimit;
            }
            printf("share!\n");
            SharingGoTo[SharingGoTo[i]] = SharingGoTo[i];
            SharingGoTo[i] = i; //シェアリング相手の登録解除
            TotalSharing++;
          }
          else if ((dSij <= 0.0))
          {
            NumOfSharing++;
            Donating[SharingGoTo[i]]++;
            Recieving[i]++;
            S_new[i] += cShare;
            S_new[SharingGoTo[i]] -= cShare;
            F(SharingGoTo[i], i) += fshare;
            F(i, SharingGoTo[i]) += fshare;
            if (F(SharingGoTo[i], i) >= Flimit)
            {
              F(SharingGoTo[i], i) = Flimit;
            }
            if (F(i, SharingGoTo[i]) >= Flimit)
            {
              F(i, SharingGoTo[i]) = Flimit;
            }
            printf("share!\n");
            SharingGoTo[SharingGoTo[i]] = SharingGoTo[i];
            SharingGoTo[i] = i; //シェアリング相手の登録解除
            TotalSharing++;
          }
        }
      }
    }
  }
}

int JudgeToGoSharing(int *pi)
{
  int j = 0, ShareDone = *pi, k = 0;
  double dSij = 0.0;

  for (j = 0; j < NumOfAgentInsideSensorRange[*pi]; j++) //誰かにシェアリングするか否かを決める
  {
    k = AgentInsideSensorRange(*pi, j);
    dSij = S_old[*pi] - S_old[k];
    //個体jが個体iのセンサレンジ内に存在し，シェアリングの条件式を満たしている時
    if ((F(*pi, k) * abs(dSij)) > ks && SharingGoTo[k] == k && abs(dSij) >= cShare)
    {
      NumOfGoSharing++;
      SharingGoTo[*pi] = k;
      SharingGoTo[k] = *pi;
      ShareDone = k;
      break;
    }
  }
  return ShareDone;
}

void Metabolism(void)
{
  int i = 0, Done = 0;
  for (i = 0; i < N; i++)
  {
    if (S_new[i] > 0.00)
    {
      S_new[i] -= cs + cv * v_old[i].get_abs();
      if (S_new[i] > Slimit)
      {
        S_new[i] = Slimit;
      }

      if (Done == 0)
      {
        Smax = S_new[i];
        Smin = S_new[i];
        Done = 1;
      }
      else
      {
        if (Smax < S_new[i])
        {
          Smax = S_new[i];
        }
        if (Smin > S_new[i])
        {
          Smin = S_new[i];
        }
      }

      if (S_new[i] <= 0.0)
      {
        SharingGoTo[SharingGoTo[i]] = SharingGoTo[i];
        SharingGoTo[i] = i; //シェアリング相手の登録解除
        //Dead[i] = 1;
        NumOfDeadCell++;
      }
    }
    S_old[i] = S_new[i];
  }
}

void Moving(void)
{
  int i = 0, j = 0, GoTo = 0, k = 0;
  double dSij = 0.0, vv_abs = 0.0;
  int ShareDone = 0;
  Vector2D Fphs; //, gl;
  Fphs.set_vec(0.0, 0.0);
  //gl.set_vec(0.0, 0.0);

  //個体間の距離を求める
  CheckDistance(); //agent iのセンサレンジ内にいる個体とagent radius内にいる個体を調べる
  for (i = 0; i < N; i++)
  {
    foodx[i] = 0.0;
    foody[i] = 0.0;
    InFood[i] = 0;
    Fphs.set_vec(0.0, 0.0);
    if (FlagRandomWalkOnly == 1) //ランダムウォーク＆衝突判定＆採餌のみの場合
    {
    }
    else //他個体とのやりとりがONな時
    {
      if (S_old[i] > 0.0) //個体iが生きていれば
      {
        ///////シェアリングするか否かを決め，座標を更新////////
        ShareDone = i;
        if (SharingGoTo[i] != i) //シェアリング相手が決まっている時
        {
          if (S_old[SharingGoTo[i]] > 0.0)
          { //シェアリング相手が生きていれば
            dSij = S_old[i] - S_old[SharingGoTo[i]];
            if (abs(dSij) < cShare)
            {                                               //相手との体力差が基準未満なら
              SharingGoTo[SharingGoTo[i]] = SharingGoTo[i]; //登録解除
              SharingGoTo[i] = i;                           //登録解除
              SharingOff(&i);
            }
            else
            {
              GoTo = SharingGoTo[i];
              GoForSharing(&i, &GoTo); //そいつに近寄っていく
            }
          }
          else //死んでいれば
          {
            SharingGoTo[SharingGoTo[i]] = SharingGoTo[i]; //登録解除
            SharingGoTo[i] = i;
            SharingOff(&i);
          }
        }
        else //シェア相手が未定な場合
        {
          ShareDone = JudgeToGoSharing(&i);
          if (ShareDone == i)
          { //Sharingをしなかったら，通常の移動
            SharingOff(&i);
          }
          else
          {
            GoForSharing(&i, &ShareDone); //このShareDoneにはシェア相手が誰か
          }
        }
      }
    }
  }
}

void Foods()
{
  double ransu_time = 0.0, Tem = 2.0;
  int ransu_x = 0, ransu_y = 0;
  int i = 0, Flag = 0;

  //エサの生成
  if (step % 100 == 0)
  {
    ransu_time = (double)rand() / RAND_MAX; //1以下の乱数
    if (ransu_time < time_th)               //閾値以上であればエサを出現させる
    {
      do
      {
        ransu_x = int(rand() % GridNum);
        ransu_y = int(rand() % GridNum);
      } while (VV[ransu_x][ransu_y].amount > 0.0);
      double rans = rand() / (RAND_MAX + 1.0);
      Tem = Food0 * rans; //
      VV[ransu_x][ransu_y].amount = Tem;
      VV[ransu_x][ransu_y].AmountInit = Tem;
      VV[ransu_x][ransu_y].radius = wide * Tem / Food0;
    }
  }
}

void keyboard(unsigned char key, int x, int y)
{
  double tmpcin;
  switch (key)
  {
  case 'q':;
    exit(0);
    break;
  case 'h':
    monitor.SetCenter(1.0, 0);
    break;
  case 'l':
    monitor.SetCenter(-1.0, 0);
    break;
  case 'j':
    monitor.SetCenter(0, 1.0);
    break;
  case 'k':
    monitor.SetCenter(0, -1.0);
    break;
  case 'z':
    monitor.SetZoom(1.1 / 1.0);
    break;
  case 'x':
    monitor.SetZoom(1.0 / 1.1);
    break;
  }
}

void Feeding(void)
{
  int i = 0, x = 0, y = 0;
  for (i = 0; i < N; i++)
  {
    if (S_old[i] > 0.00)
    {
      if (InFood[i])
      {
        x = foodx[i];
        y = foody[i];
        if (pow_absr[i] <= (AgentRadius + VV[x][y].radius) * (AgentRadius + VV[x][y].radius) && VV[x][y].amount > 0.0)
        {
          if (VV[x][y].amount <= c_eat)
          {
            S_new[i] += VV[x][y].amount;
            VV[x][y].amount = 0.0;
          }
          else
          {
            S_new[i] += c_eat;
            VV[x][y].amount -= c_eat;
          }
        }
      }
    }
  }
}

void Grooming(void)
{
  int i = 0, j = 0, GroomDone[N][N] = {};
  double FGroom = 1.0;
  int Done = 0;

  for (i = 0; i < N; i++)
  {
    FSum[i] = 0.0; //他者への好感度の総和
    GroomFlag[i] = 0;
    for (j = 0; j < N; j++)
    {
      GroomDone[i][j] = 0;
      if (i != j && S_old[i] > 0.0 && S_old[j] > 0.0)
      {
        //好感度の更新
        FSum[i] += F(i, j);
      }
    }
  }

  for (i = 0; i < N; i++)
  {
    if (S_old[i] > 0.0)
    {
      for (j = 0; j < N; j++)
      {
        if (i != j && S_old[j] > 0.0)
        {
          F(i, j) = (1 - eps1) * F(i, j);
          if (DistanceBetweenIJ[i][j] <= SensorRange && GroomDone[i][j] == 0)
          { //Grooming実行
            GroomDone[i][j] = 1;
            GroomDone[j][i] = 1;
            GroomFlag[i] = 1; //表示用
            GroomFlag[j] = 1;
            FGroom = k2 - FSum[i]; //個体iのぶん
            if (FGroom < 0.0)
            {
              FGroom = 0.0000000000000;
            }
            F(i, j) += cgroom * FGroom;
            //個体jのぶん
            FGroom = k2 - FSum[j];
            if (FGroom < 0.0)
            {
              FGroom = 0.000000000000000;
            }
            F(j, i) += cgroom * FGroom;
          }
          if (F(i, j) > 1.0)
          {
            F(i, j) = 1.0;
          }
          if (F(j, i) > 1.0)
          {
            F(j, i) = 1.0;
          }
          //好感度Fの最大値と最小値を調べる
          if (Done == 0)
          {
            Fmax = F(i, j);
            Fmin = F(i, j);
            Done = 1;
          }
          else
          {
            if (Fmax < F(i, j))
            {
              Fmax = F(i, j);
            }
            if (Fmin > F(i, j))
            {
              Fmin = F(i, j);
            }
          }
        }
      }
    }
  }
}

void idle(void)
{
  glutSetWindow(winid);
  //glutKeyboardFunc(keyboard);
  glutPostRedisplay();
}

void capture(long long int *pstep)
{
  char filepath[100]; //= "./MovieDir/output.png";
  sprintf(filepath, "./MovieDir/%d.png", *pstep);
  png_bytep raw1D;
  png_bytepp raw2D;
  int i;
  int width = glutGet(GLUT_WINDOW_WIDTH);
  int height = glutGet(GLUT_WINDOW_HEIGHT);

  // 構造体確保
  FILE *fp = fopen(filepath, "wb");
  png_structp pp = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_infop ip = png_create_info_struct(pp);
  // 書き込み準備
  png_init_io(pp, fp);
  png_set_IHDR(pp, ip, width, height,
               8,                   // 8bit以外にするなら変える
               PNG_COLOR_TYPE_RGBA, // RGBA以外にするなら変える
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  // ピクセル領域確保
  raw1D = (png_bytep)malloc(height * png_get_rowbytes(pp, ip));
  raw2D = (png_bytepp)malloc(height * sizeof(png_bytep));
  for (i = 0; i < height; i++)
    raw2D[i] = &raw1D[i * png_get_rowbytes(pp, ip)];
  // 画像のキャプチャ
  glReadBuffer(GL_FRONT);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // 初期値は4
  glReadPixels(0, 0, width, height,
               GL_RGBA,          // RGBA以外にするなら変える
               GL_UNSIGNED_BYTE, // 8bit以外にするなら変える
               (void *)raw1D);
  // 上下反転
  for (i = 0; i < height / 2; i++)
  {
    png_bytep swp = raw2D[i];
    raw2D[i] = raw2D[height - i - 1];
    raw2D[height - i - 1] = swp;
  }
  // 書き込み
  png_write_info(pp, ip);
  png_write_image(pp, raw2D);
  png_write_end(pp, ip);
  // 開放
  png_destroy_write_struct(&pp, &ip);
  fclose(fp);
  free(raw1D);
  free(raw2D);

  //printf("write out screen capture to '%s'\n", filepath);
}

void Display(void)
{
  int i = 0, j = 0, q = 0;
  char str[20];
  double vv_abs = 0.0, colors = 0.0;
  int food_x = 0, food_y = 0;

  glClear(GL_COLOR_BUFFER_BIT);
  glLoadIdentity();
  //シミュレーション環境の境界を表示
  monitor.SetAllColor(0.0, 0.0, 0.00);
  double LL = L + AgentRadius;
  monitor.DrawLine(-LL, -LL, -LL, LL, 0.1);
  monitor.DrawLine(-LL, LL, LL, LL, 0.1);
  monitor.DrawLine(LL, LL, LL, -LL, 0.1);
  monitor.DrawLine(LL, -LL, -LL, -LL, 0.1);
  //エサの表示
  for (i = 0; i < GridNum; i++)
  {
    for (j = 0; j < GridNum; j++)
    {
      colors = exp(-2.0 * VV[i][j].amount);
      if (colors < 1.0) //エサが存在していれば
      {
        monitor.SetAllColor(colors, colors, colors);
        monitor.DrawCircle((double)L * ((double)i - 100.0) / 100.0, (double)L * ((double)j - 100.0) / 100.0, VV[i][j].radius);
      }
    }
  }
  for (j = 0; j < N; j++)
  {
    //FSum[j] = 0.0;
    if (S_old[j] > 0.0)
    {
      for (i = 0; i < N; i++)
      {
        if (i != j)
        {
          if (S_old[i] > 0.0)
          {
            if (F(i, j) > FThreshHigh)
            {
              monitor.SetAllColor(0.0, 0.0, 1.0);
              monitor.DrawLine(r_new[j].get_x(), r_new[j].get_y(), r_new[i].get_x(), r_new[i].get_y(), 0.1);
            }
          }
        }
      }
      //センサレンジとprivate zoneの表示
      //SharingFlag[j] = 0;
      monitor.SetAllColor(0.0, 0.0, 0.0);
      if (SharingGoTo[j] == j) //シェアしなければ
      {
        monitor.SetAllColor(0.0, (double)GroomFlag[j], 0.0);
      }
      else //シェアするなら
      {
        monitor.SetAllColor(1.0, 0.0, 0.0);
      }
      GroomFlag[j] = 0;
      q = 1;
      for (i = -q; i < q; i++)
      {
        //センサレンジ
        monitor.DrawLine(r_new[j].get_x() + (double)(SensorRange * i / q), r_new[j].get_y() + sqrt(pow(SensorRange, 2) - pow(SensorRange * i / q, 2)), r_new[j].get_x() + (double)(SensorRange * (i + 1) / q), r_new[j].get_y() + sqrt(pow(SensorRange, 2) - pow(SensorRange * (i + 1) / q, 2)), 0.1); //y=sqrt(1-x^2)
        monitor.DrawLine(r_new[j].get_x() + (double)(SensorRange * i / q), r_new[j].get_y() - sqrt(pow(SensorRange, 2) - pow(SensorRange * i / q, 2)), r_new[j].get_x() + (double)(SensorRange * (i + 1) / q), r_new[j].get_y() - sqrt(pow(SensorRange, 2) - pow(SensorRange * (i + 1) / q, 2)), 0.1);
      }

      if (S_new[j] > 0.8)
      {
        monitor.SetAllColor(0.0, 0.0, 1.0);
      }
      else if (S_new[j] > 0.4)
      {
        monitor.SetAllColor(0.0, 1.0, 0.0);
      }
      else
      {
        monitor.SetAllColor(1.0, 0.0, 0.0);
      }
      monitor.DrawCircle(r_new[j].get_x(), r_new[j].get_y(), AgentRadius);
    }
  }
  monitor.SetAllColor(0.0, 0.0, 0.00);
  //パラメータ値を画面表示
  double y = 0.9, x = 0.0, y0 = 0.05;
  sprintf(str, "cs=%f", cs);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "cv=%f", cv);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "time_th=%f", time_th);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "eps1=%f", eps1);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "count1=%ld", count1);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "count2=%ld", count2);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "k2=%f", k2);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "ks=%f", ks);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "t=%lf", t);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "Death=%d", NumOfDeadCell);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "Fmax=%lf", Fmax);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "Fmin=%lf", Fmin);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "Smax=%.3f", Smax);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, "Smin=%.3f", Smin);
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, " NumofSharing=%d", NumOfSharing);
  NumOfSharing = 0;
  monitor.String(x, y, str);
  y -= y0;
  sprintf(str, " NumofCollision=%d", NumOfCollision);
  monitor.String(x, y, str);
  NumOfCollision = 0;
  y -= y0;
  sprintf(str, "NumofGoSharing=%d", NumOfGoSharing);
  monitor.String(x, y, str);
  //TotalSharing += NumOfGoSharing;
  NumOfGoSharing = 0;
  glFlush();
  glutSwapBuffers();
}

void Simulating(void)
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  int i = 0, j = 0, p = 0, q = 0;
  double M, dHij, dHji;

  //移動
  for (p = 0; p < DisplayInterval; p++)
  {
    for (q = 0; q < 10; q++) //100回まとめてやる
    {
      Moving();
    }
    //エサ食べ
    Feeding();
    if (!FlagRandomWalkOnly)
    {
      //毛づくろい&好感度の減衰，好感度の総和を計算
      Grooming();
    }
    DoSharing();
    //代謝計算
    Metabolism();

    //エサの追加
    if (FlagFood && step % 100 == 0)
    {
      Foods();
    }
    step++;
  }
  t = (double)(step)*dt;
  //画面表示
  Display();

  if (FlagDataSave && step % SaveDataToFilesIntervals == 0)
  {
    //////データを配列に格納//////
    sprintf(Whole[DataCount], "%.3f,%d\n", t, N - NumOfDeadCell);
    DataCount++;
    //////データをファイルに格納
    if (DataCount == (StrNum - 1))
    {
      printf("データをファイルに格納 \n");
      fpWhole = fopen(strWhole, "a");
      printf("%s\n", strWhole);
      if (fpWhole == NULL)
      {
        printf("%s\n", strFolderParameterSetCount);
        printf("%s のオープン失敗\n", strWhole);
        exit(EXIT_FAILURE); // 強制的に異常終了。exit(1)でも可
      }
      for (i = 0; i <= DataCount; i++)
      {
        fprintf(fpWhole, "%s", Whole[i]);
      }
      fclose(fpWhole);
      DataCount = 0;
    }
  }

  if (N == NumOfDeadCell || t == EndTime)
  {
    fpWhole = fopen(strWhole, "a");
    printf("%s\n", strWhole);
    if (fpWhole == NULL)
    {
      printf("%s のオープン失敗\n", strWhole);
      exit(EXIT_FAILURE); // 強制的に異常終了。exit(1)でも可
    }
    //データをファイルに格納
    for (i = 0; i < DataCount; i++)
    {
      fprintf(fpWhole, "%s", Whole[i]);
    }
    fclose(fpWhole);

    TotalSharing = 0;
    if ((ParameterSetCount == EndSetCount && FileCount == ToEndFileCount) || save_flag == 1)
    {
      exit(0);
    }
    init(); //初期化＆再スタート
  }
  //1000ステップごとに画像を保存

  //monitor.SavePPMData();
  capture(&step);
}

void mouse(int button, int state, int x, int y)
{
  switch (button)
  {
  case GLUT_LEFT_BUTTON:
    if (state == GLUT_DOWN)
    {
      glutIdleFunc(0);
      std::cout << "left: on" << std::endl;
    }
    else
    {
      glutIdleFunc(idle);
      std::cout << "left: off" << std::endl;
    }
    break;

  case GLUT_MIDDLE_BUTTON:
    if (state == GLUT_DOWN)
    {
      glutIdleFunc(0);
      std::cout << "middle: on" << std::endl;
    }
    else
    {
      glutIdleFunc(idle);
      std::cout << "middle: off" << std::endl;
    }
    break;

  case GLUT_RIGHT_BUTTON:
    if (state == GLUT_DOWN)
    {
      glutIdleFunc(0);
      std::cout << "right: on" << std::endl;
    }
    else
    {
      glutIdleFunc(idle);
      std::cout << "right: off" << std::endl;
    }
    break;
  }
}

void resize(int w, int h)
{
  glViewport(0, 0, w, h);
  glLoadIdentity();
  glOrtho(-w / 0.0, w / 20.0, -h / 20.0, h / 20.0, -3.0, 3.0);
}

void OpenGL_init(int *argcp, char **argv)
{
  init(); //初期条件を設定

  glutInit(argcp, argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

  glutInitWindowSize(monitor.GetWindowSize(Monitor::X), monitor.GetWindowSize(Monitor::Y));
  glutInitWindowPosition(10, 100);
  winid = glutCreateWindow("simulation");

  glutDisplayFunc(Simulating);
  glutReshapeFunc(resize);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glClearColor(1.0, 1.0, 1.0, 1.0);
}

void monitor_init()
{
  monitor.SetWindowSize(100, 75);
  monitor.SetMovieMode(1);
  monitor.SetMovieName("./MovieDir/temp_");
  monitor.SetZoom(zoom);
}

int main(int argc, char *argv[])
{
  int i, j, i_dim;

  monitor_init();
  std::cout << "monitor init OK" << std::endl;
  OpenGL_init(&argc, argv);
  std::cout << "OpenGL init OK" << std::endl;
  glutIdleFunc(idle);
  glutMainLoop(); //無限ループ
  return 0;
}
