
/* cc car-navi.c -g -O2 -Wall -Wno-unused-result -o car-navi -I/usr/include/freetype2 -lftgl -lglfw -lGLU -lGL -lX11 -lXrandr -lm 
 */



// 工夫点について 
// デフォルトのコメントと見分けるために新しく書いたコメントはすべて　//　で記述
//
// 大まかな工夫した点は以下
// キーボードで拡大縮小、画面表示の切り替え、視点変更を可能にした
// また加速と一時停止を追加した
// 通過経路を黄色で表示
// 移動物体の先端を、進行方向の向きに常に移動するようにした
// 移動物体を動かすのではなく、地図自体を平行移動させるようにした
// 移動物体の進行方向が常に上向きになるように、地図の座標を回転させ、新しい座標を計算、地図の表示を出来るようにした
// 移動物体の進行方向をわかりやすくするために形状を工夫
// 到着後に移動物体が白に変わるように工夫
// search_crossでローマ字だけでなく、交差点番号でも探せるように工夫した
// カーナビの画面左半分でルートを表示し、どこの交差点の間にいるかを矢印の色でわかるようにした
// またスタート地点とゴール地点で、"Start","Arrival!!"と表示されるようにした。
// キーボードが押された場合、何を押したか表示されるようにした
// 
//
// また適宜コメントを書いたので参照お願いします


#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>


#include <unistd.h>
#include <GL/glfw.h>
#include <FTGL/ftgl.h>


#define CrossingNumber 100  /* 交差点数=100 */
#define MaxName         50  /* 最大文字数50文字(半角) */


/* 追加するマクロ定義 */
#define PATH_SIZE     100  /* 経路上の最大の交差点数 */
#define MARKER_RADIUS 0.2  /* マーカーの半径 */

/* 座標変換マクロの定義 */
#define ORIGIN_X      1.0
#define ORIGIN_Y     -1.0
double REAL_SIZE_X = 14.0; // 拡大縮小させるためdefine→doubleに工夫
double REAL_SIZE_Y = 14.0;
double VIEW_POINT_X = 0.0; // 視点変更のためにVIEW_POINTを定義
double VIEW_POINT_Y = 0.0;
double MOVE = 0.2;
double delta_x = 0.0;
double delta_y = 0.0;
int d, a;			// dは画面表示、aは加速するときに使用								
int w = 1;			// wは一時停止の時に使用
int start, goal;
int vehicle_pathIterator = 0;     /* 移動体の経路上の位置 (何個目の道路か) */

#ifndef FONT_FILENAME
/* 演習サーバに用意されているフォントのファイル名 */
#define FONT_FILENAME "/usr/share/fonts/truetype/takao-gothic/TakaoGothic.ttf"
#endif
static FTGLfont *font; /* 読み込んだフォントを差すポインタ */




typedef struct {
  double x, y;             /* 位置 x, y */
} Position;                /* 位置を表す構造体 */

typedef struct {
  int id;                /* 交差点番号 */
  char id_c[MaxName];    // idから検索を可能にするため定義
  Position pos;          /* 位置を表す構造体 */
  Position pos_c;	 // 新しい位置を表す構造体を定義
  Position pos_path;
  double wait;           /* 平均待ち時間 */
  char jname[MaxName];   /* 交差点名 */
  char ename[MaxName];   /* 交差点名 */
  int points;            /* 交差道路数 */
  int next[5];           /* 隣接する交差点番号 */
  double distance;       /* 基準交差点までの距離：追加 */
  int minpath;
} Crossing;

Crossing cross[CrossingNumber];

int map_read(char *filename)
{
    FILE *fp;
    int i, j;
    int crossing_number;          /* 交差点数 */

    fp = fopen(filename, "r");
    if (fp == NULL) {
        perror(filename);
        return -1;
    }

    /* はじめに交差点数を読み込む */
    fscanf(fp, "%d", &crossing_number);

    for (i = 0; i < crossing_number; i++) {

        fscanf(fp, "%d,%lf,%lf,%lf,%[^,],%[^,],%d",
                     &(cross[i].id), &(cross[i].pos.x), &(cross[i].pos.y),
                     &(cross[i].wait), cross[i].jname,
                     cross[i].ename, &(cross[i].points));

        for (j = 0; j < cross[i].points; j++) {
            fscanf(fp, ",%d", &(cross[i].next[j]));
        }

    }
    fclose(fp);

    /* ファイルから読み込んだ交差点数を返す */
    return crossing_number;
 // いままでとかわりません。
}

/* 距離を表示できるよう改造してます */
void print_cross(int i)
{
  int j;
  printf("交差点番号:%2d, 座標(%5.2lf,%5.2lf), 名前: %s ( %s ),",
	 cross[i].id, cross[i].pos.x, cross[i].pos.y,
	 cross[i].jname,cross[i].ename);
   
  printf("\n    目的地までの距離 %5.1lf 待ち時間:%5.1lf, 隣接交差点 :%d個 ( ", 
	 cross[i].distance,cross[i].wait, cross[i].points);  /* ここを改造 */
  for(j=0; j<cross[i].points; j++)         /* 交差道路数だけ繰り返し */
    printf("%d ", cross[i].next[j]);
  
  printf(")\n\n");
}



/* num個表示 = 表示数制限可能 */
void print_cross_list(int num)  
{
  int i;

    for (i = 0; i < num; i++)
        print_cross(i);
  // 略
/* これまでに作成した print_cross_list をそのまま拝借 */

}




// 現在地と目的地を入力する際にローマ字だけでなく、交差点番号でも判定できるように工夫
int search_cross(int num)
{
    int i;
    char buff[256];
    int f = -1;      /* 見つかったかどうかのフラグ */
    printf("交差点名を入力してください (ローマ字または交差点番号): ");
    scanf("%s", buff);

    for (i = 0; i < num; i++) sprintf(cross[i].id_c, "%d", i);						// 交差点番号がint型で比較できないので、新しくcross[i].id_cにchar型として交差点番号を代入

    for (i = 0; i < num; i++) { /* データを最後までループ　*/
	  if (strcmp(cross[i].ename,buff) == 0 || strcmp(cross[i].id_c, buff) == 0) {  /* 一致したら*/	// ローマ字、もしくは交差点番号で等しいものを探す
            printf("交差点名 %s(%s)  座標 %.2lf,%.2lf  駅からの距離 %.2lf\n",
            cross[i].jname,cross[i].ename,
            cross[i].pos.x,cross[i].pos.y,
            hypot(cross[i].pos.x, cross[i].pos.y));
            /* hypot は sqrt(x*x+y*y) を計算します */
            f = i;
        }
    }
    if (f == -1)
        puts("見つかりませんでした");
    return f;
}



/* 交差点 a と b の間の「距離」を与える */
double distance(int a, int b)
{
  return hypot(cross[a].pos.x-cross[b].pos.x,   /* この式を変えると */
	       cross[a].pos.y-cross[b].pos.y);  /* 評価がかわります */
}




void dijkstra(int crossing_number,int target)
{ 
  int i, j, n;
  double d;
  double min_distance;
  int min_cross = 0;
  int done[CrossingNumber];
  
  for(i=0; i<crossing_number; i++)
    {
      cross[i].distance = 1e100;
      done[i]=0;
      cross[i].minpath = 0;
    }
  
  cross[target].distance = 0;
  
  for(i=0; i<crossing_number; i++)
    {
      //最も距離数値の小さな未確定交差点を選定
      min_distance = 1e100;
      min_cross = -1;
      for(j=0; j<crossing_number; j++)
	{
	  if(done[j] == 0 && cross[j].distance < min_distance){
	    min_distance = cross[j].distance;
	    min_cross = cross[j].id;
	  }
	}
      done[min_cross] = 1;
      cross[min_cross].distance = min_distance;
      
      for(j=0; j<cross[min_cross].points; j++){
	n = cross[min_cross].next[j];
	d = distance(min_cross, n) + min_distance;
	if(cross[n].distance > d) {
	  cross[n].distance = d;
	  cross[n].minpath = min_cross;
	}
      }  
    }
}

// pickup_pathをよりシンプルに改造
int pickup_path(int crossing_number,int start,int goal,
		int path[],int maxpath)
{
  int i;

  path[0]=start;
  i=1;

  for(;;)
    {
      path[i] = cross[path[i-1]].minpath;
      i++;
      if(path[i-1] == goal){break;}
    }
  path[i] = -1;

  return 0;
}



/* 文字列を描画 */
void draw_outtextxy(double x, double y, char const *text) {
    double const scale = 0.01;

    glPushMatrix();
    glTranslated(x, y, 0.0);
    glScaled(scale, scale, scale + 100);
    ftglSetFontFaceSize(font, 14, 14);		
    ftglRenderFont(font, text, FTGL_RENDER_ALL);
    glPopMatrix();
}



  
/* 円を描画 */
void draw_circle(double x, double y, double r) {
    int const N = 12;             /* 円周を 12分割して線分で描画することにする */
    int i;

    glBegin(GL_LINE_LOOP);
    for (i = 0; i < N; i++)
        glVertex2d(x + cos(2 * M_PI * i / N) * r,
                   y + sin(2 * M_PI * i / N) * r);
    glEnd();
}

// ルートを表示する際に使う矢印を定義
void draw_vector(double x, double y) {
     
    glBegin(GL_POLYGON);
    glVertex2d(x, y);
    glVertex2d(x + 0.3, y);
    glVertex2d(x + 0.15, y - 0.08);
    glEnd();
}


// カーナビの画面左半分にルートが表示されるよう関数を定義
void hyouji(double x, double y, int *path, int i) {

    int s;
    int t = 0;
    int u = 0;

    for(s = 0; path[i + s] != -1; s++) t = t + 1;						// 直前に通過した交差点から目的地までの通過する交差点数を数える
    for(s = 0; path[s] != -1; s++) u = u + 1;							// 目的地までの通過する交差点の個数を数える
    if(t < 10 && u > 9) t = 9;									// 表示が画面から飛び出さないように調整
    
	    
    glColor3d(0.2, 0.2, 0.2);
    glBegin(GL_POLYGON);
    glVertex2d( x, y);
    glVertex2d( x + 1.4, y);
    if(t < 11 && u > 10) { 									// 交差点数が９を超える場合、制限して表示
    glVertex2d( x + 1.4, y - t*0.3 - 0.35);
    glVertex2d( x, y - t*0.3 - 0.35);
    } else {											// 交差点数が９を超えない場合、制限せずに表示
    glVertex2d( x + 1.4, y - u*0.3 - 0.35);
    glVertex2d( x, y - u*0.3 - 0.35);
    }
    glEnd();

    if(t != 9 && u > 9) {									// 交差点数が１０以上で、直前に通過した交差点から目的地までの交差点数が９でない時の表示
      for(s = 0; path[i + s] != -1 && s <9; s++)  {						// ルート表示が画面の外に飛び出さないようにfor文を工夫						
        if(i + s == 0) {
	   glColor3d(1.0, 1.0, 1.0);
	   draw_outtextxy(x + 0.1, y - 0.1 - 0.15, "Start");					// 出発地点の上に"Start"と表示
	}
	glColor3d(1.0, 0.0, 0.0);
	draw_outtextxy(x + 0.1, y - 0.4 - 0.3*s, cross[path[i + s]].jname);			// 数を調整して文字を表示
	if(s < 8) {										// 矢印が文字列数を超えないよう工夫
	   if( i + s == i) glColor3d(1.0, 1.0, 0.0);
	   else glColor3d(1.0, 1.0, 1.0);
	   draw_vector(x + 0.1, y - 0.45 - 0.3*s);						// 文字列の間に調整して矢印を表示
	}
      }
    } else if(t == 9 && u > 9){									// 交差点数が１０以上で、直前に通過した交差点から目的地までの交差点数が９の時の表示
      for(s = u - t; path[s] != -1; s++)  {							// ルート表示が画面の外に飛び出さないようにfor文を工夫
	glColor3d(1.0, 0.0, 0.0);
	draw_outtextxy(x + 0.1, y - 0.4 - 0.3*(s - u + t), cross[path[s]].jname);		// 数を調整して文字列を表示
	if(path[s + 1] != -1) {									// 矢印が文字列を超えないよう工夫
	   if( s == i) glColor3d(1.0, 1.0, 0.0);						// 走行区間の矢印の色を黄色に変える
	   else glColor3d(1.0, 1.0, 1.0);							// それ以外の矢印は白
	   draw_vector(x + 0.1, y - 0.45 - 0.3*(s-u+t));					// 文字列の間に調整して矢印を表示
	}
      }
      glColor3d(1.0, 1.0, 1.0);
      if(path[i + 1] == -1) draw_outtextxy(x + 0.1, y - 0.25 - 0.3*t, "Arrival!!");		// 到着した時に"Arrival!!"と表示
    } else if(u <= 9) {										// 交差点数が１０未満の時の表示
      glColor3d(1.0, 1.0, 1.0);
      draw_outtextxy(x + 0.1, y - 0.1 - 0.15, "Start");						// 出発地点の上に"Start"と表示
      for(s = 0; path[s] != -1; s++)  {								
	glColor3d(1.0, 0.0, 0.0);
	draw_outtextxy(x + 0.1, y - 0.4 - 0.3*s, cross[path[s]].jname);				// 文字列を表示
	if(path[s + 1] != -1) {									// 矢印が文字列を超えないように工夫
	   if( s == i) glColor3d(1.0, 1.0, 0.0);						// 走行区間の矢印の色を黄色に変える
	   else glColor3d(1.0, 1.0, 1.0);							// それ以外の矢印は白
	   draw_vector(x + 0.1, y - 0.45 - 0.3*s);						// 文字列の間に矢印が表示されるように工夫
	}
      }
      glColor3d(1.0, 1.0, 1.0);
      if(path[i + 1] == -1) draw_outtextxy(x + 0.1, y - 0.25 - 0.3*u, "Arrival!!");		// 到着した時に"Arrival!!"と表示
   }
}


// キーボードを押すと拡大縮小、また画面表示を変えることができるように工夫
void GLFWCALL KeyBoard(int key, int action)
{
     printf("Key \'%c\' is %s!\n", key, action == GLFW_PRESS ? "pressed" : "released");

     if(key == 'L' && REAL_SIZE_Y > 1) {				// Lが押された時、拡大表示できるように工夫
        REAL_SIZE_X = REAL_SIZE_X - 1.0;
        REAL_SIZE_Y = REAL_SIZE_Y - 1.0;
     }

     if(key == 'S') {				// Sが押された時、縮小表示できるように工夫
        REAL_SIZE_X = REAL_SIZE_X + 1.0;
        REAL_SIZE_Y = REAL_SIZE_Y + 1.0;
     }

     if(key == 'F') {				// Fが押された時、元の縮図に直して表示できるように工夫
        REAL_SIZE_X = 14.0;
        REAL_SIZE_Y = 10.0;
     }


     if(key == 'C' && action == GLFW_PRESS) {
        d = d * (-1);				// Cボタンひとつで画面表示できるように工夫
     }

     if(key == 'A' && action == GLFW_PRESS) {
        a = a * (-1);				// 表示物体が加速される
     }

     if(key == 'W' && action == GLFW_PRESS) {
        w = w * (-1);				// 表示物体が一時停止される
     }



     if(key == 'R' && action == GLFW_PRESS) {
        VIEW_POINT_X = 0.0;
        VIEW_POINT_Y = 0.0;
	delta_x = 0.0;
	delta_y = 0.0;
     }

     // 矢印キーで視点変更できるように工夫
     // ルート表示が動かないように、deltaで移動分を代入
     switch(key) {				
	case GLFW_KEY_LEFT:
	VIEW_POINT_X += MOVE;
	delta_x -= MOVE;
	break;

	case GLFW_KEY_RIGHT:
	VIEW_POINT_X -= MOVE;
	delta_x += MOVE;
	break;

	case GLFW_KEY_UP:
	VIEW_POINT_Y -= MOVE;
	delta_y += MOVE;
	break;

	case GLFW_KEY_DOWN:
	VIEW_POINT_Y += MOVE;
	delta_y -= MOVE;
	break;	
     }
	
}




// 円を塗りつぶす関数を新たに導入
void draw_circle_all(double x, double y, double r) {
    int const N = 12;             /* 円周を 12分割して線分で描画することにする */
    int i;

    glBegin(GL_POLYGON);			//　円を塗りつぶすためにGL_POLYGONを使用
    for (i = 0; i < N; i++)
        glVertex2d(x + cos(2 * M_PI * i / N) * r,
                   y + sin(2 * M_PI * i / N) * r);
    glEnd();
}




// 進行方向がわかりやすいように形を工夫、矢印先端が進行方向に向かって動くようになっている
void draw_triangle_0(double x, double y, double theta, double r) {

    glBegin(GL_POLYGON);
    glVertex2d(r*cos(theta) + x, r*sin(theta) + y);
    glVertex2d(r*cos(theta + 2.61799) + x, r*sin(theta + 2.61799) + y);					// 5pi/6で回転
    glVertex2d(x + 0.5 * r * cos(theta + 3.141592), y + 0.5 * r * sin(theta + 3.141592));		// pi方向に基準点から少し延長
    glVertex2d(r*cos(theta + 3.66519) + x, r*sin(theta + 3.66519) + y);					// 7pi/6で回転  

    glEnd();
}



// 三角形ではなく、地図が動くように新しい関数を導入　現在地が画面中央にくるように平行移動させている
void map_motion_0(int crossing_number, double x_standard, double y_standard, double x, double y, int *path) {
//    double x_standard = 1.7;			// 画面中心のx座標
//    double y_standard = -1.0;			// 画面中心のy座標
    int i, j;
    double x0, y0;
    double x_differ;				
    double y_differ;

    // すべての座標を現在地が画面中心に来るように平行移動
    x_differ = x_standard - x;			
    y_differ = y_standard - y;		

   
    for(i=0; i<crossing_number; i++)  {
	cross[i].pos_path.x = cross[i].pos.x + x_differ;		// Warningが出ないように再定義
	cross[i].pos_path.y = cross[i].pos.y + y_differ;   
    }
	
	

    // すべての座標を現在地が画面中心に来るように平行移動しmapを描く
    for(i=0; i<crossing_number; i++)  {	

	/* 交差点を表す円を描く */
        glColor3d(1.0, 0.5, 0.5);
        draw_circle(cross[i].pos_path.x, cross[i].pos_path.y, 0.1);

        /* 交差点から伸びる道路を描く */
        glColor3d(1.0, 1.0, 1.0);
        glBegin(GL_LINES);
        for(j = 0; j < cross[i].points; j++) {
	    // 
            x0 = cross[ cross[i].next[j] ].pos.x + x_differ;
            y0 = cross[ cross[i].next[j] ].pos.y + y_differ;		
            glVertex2d(cross[i].pos_path.x, cross[i].pos_path.y);
            glVertex2d(x0, y0);
        }
        glEnd();
    }

    // 通過する交差点を塗りつぶす
    for(i=0; path[i] != -1; i++) {		// path[i] が-1になるまでfor文を回す
	glColor3d(1.0, 1.0, 0.0);		// 塗りつぶす色は黄色で設定
	draw_circle_all(cross[path[i]].pos.x + x_differ, cross[path[i]].pos.y + y_differ, 0.1);		
	}

    //  決められたルートの線をつなぐ 線を上に出すために順番を工夫	
    glBegin(GL_LINE_STRIP);      
    for(i=0; path[i] != -1; i++) {		// path[i] が-1になるまでfor文を回す
	glColor3d(1.0, 1.0, 0.0);		// 塗りつぶす色は黄色で設定
	glVertex2d(cross[path[i]].pos.x + x_differ, cross[path[i]].pos.y + y_differ);			
	}
    glEnd(); 

    for(i=0; path[i] != -1; i++) {		// path[i] が-1になるまでfor文を回す	
	/* 交差点の名前を描く */      			// 文字を全面に出すために順番を工夫
        glColor3d(1.0, 0.5, 0.5);			// 文字の色は赤で設定
        draw_outtextxy(cross[path[i]].pos_path.x, cross[path[i]].pos_path.y, cross[path[i]].jname);	
	}
}







// 進行方向がわかりやすいように形を工夫、矢印先端が常に真上になるように関数を導入
void draw_triangle_1(double x, double y, double r) {
    glBegin(GL_POLYGON);
    glVertex2d(x + r * cos(1.57079), y + r * sin(1.57079));						// pi/2方向
    glVertex2d(x + r * cos(4.18878), y + r * sin(4.18878));						// 4pi/3方向
    glVertex2d(x + 0.5 * r*cos(1.57079 + 3.141592), y + 0.5 * r *sin(1.57079 + 3.141592));		// 3pi/2方向に基準点から少し延長
    glVertex2d(x + r * cos(5.23598), y + r * sin(5.23598));						// 5pi/3方向
    glEnd();
}



// mapの座標変換(任意座標からの回転)、進路が常に上向きになるように変換するような関数を導入
// 座標を書き換えることで、文字表示が常に右向きとなるよう工夫した
void map_change(int crossing_number, double a0, double b0, double a1, double b1) {
    double s0, t0;
    double theta1, theta2;
    int  i;
    double dist;

    theta1 = 1.57079 - atan2(b1-b0, a1-a0);				// 進行方向が常に上向きになるような、回転角を弧度法で求める

    for(i=0; i<crossing_number; i++) {	
	s0 = cross[i].pos.x;
	t0 = cross[i].pos.y;

	theta2 = atan2(t0-b0, s0-a0);					// 基準の交差点から任意の交差点までの角度を弧度法で求める

	dist = hypot(s0-a0, t0-b0);					// 基準の交差点から任意の交差点までの距離を求める
	
	cross[i].pos_c.x = a0 + dist * cos(theta1 + theta2);		// 進路が常に上向きになるように回転させた(x,y)座標をcross[i].pos_cに代入
	cross[i].pos_c.y = b0 + dist * sin(theta1 + theta2);
    }
}



// 三角形ではなく、地図が動くように新しい関数を導入　現在地が画面中央にくるように平行移動させており、回転後の座標を使用している
void map_motion_1(int crossing_number, double x_standard, double y_standard, double x, double y, double a0, double b0, double a1, double b1, int *path) {
//    double x_standard = 1.0;				// 画面中心のx座標を工夫
//    double y_standard = -1.0;				// 画面中心のy座標を工夫
    int i, j;
    double x0, y0;
    double x_differ;
    double y_differ;

    // すべての座標を現在地が画面中心に来るように平行移動
    x_differ = x_standard - x;
    y_differ = y_standard - y;


    map_change(crossing_number, a0, b0, a1, b1);	// 上で定義したmap_change関数を使用し、cross[i].pos_cに回転後の座標を代入


      for(i=0; i<crossing_number; i++)  {
	cross[i].pos_path.x = cross[i].pos_c.x + x_differ;
	cross[i].pos_path.y = cross[i].pos_c.y + y_differ;
      }


      for(i=0; i<crossing_number; i++)  {
	/* 交差点を表す円を描く */
        glColor3d(1.0, 0.5, 0.5);
        draw_circle(cross[i].pos_path.x, cross[i].pos_path.y, 0.1);


        /* 交差点から伸びる道路を描く */
        glColor3d(1.0, 1.0, 1.0);
        glBegin(GL_LINES);
        for(j = 0; j < cross[i].points; j++) {
            x0 = cross[ cross[i].next[j] ].pos_c.x + x_differ;
            y0 = cross[ cross[i].next[j] ].pos_c.y + y_differ;
            glVertex2d(cross[i].pos_path.x, cross[i].pos_path.y);
            glVertex2d(x0, y0);
        }
        glEnd();
    }

    // 決められたルートを塗りつぶす
    for(i=0; path[i] != -1; i++) {			// path[i] が-1になるまでfor文を回す
	glColor3d(1.0, 1.0, 0.0);			// 塗りつぶす色は黄色で設定
	draw_circle_all(cross[path[i]].pos_c.x + x_differ, cross[path[i]].pos_c.y + y_differ, 0.1);
    }
	
    //  決められたルートを線でつなぐ 線を上に出すために順番を工夫
    glBegin(GL_LINE_STRIP);      
    for(i=0; path[i] != -1; i++) {			// path[i] が-1になるまでfor文を回す
	glColor3d(1.0, 1.0, 0.0);			// 塗りつぶす色は黄色で設定
	glVertex2d(cross[path[i]].pos_c.x + x_differ, cross[path[i]].pos_c.y + y_differ);
	}
    glEnd(); 

    for(i=0; path[i] != -1; i++) {			// path[i] が-1になるまでfor文を回す
	/* 交差点の名前を描く */      			// 文字を全面に出すために順番を工夫
        glColor3d(1.0, 0.5, 0.5);			// 文字の色は赤で設定
        draw_outtextxy(cross[path[i]].pos_path.x, cross[path[i]].pos_path.y, cross[path[i]].jname);
	}


}






int main(void)
{
  int crossing_number;                          /* 交差点数 */
//  int goal,start;
  int path[20];
  int i;
//  int vehicle_pathIterator = 0;     /* 移動体の経路上の位置 (何個目の道路か) */
  int vehicle_stepOnEdge = 0;       /* 移動体の道路上の位置 (何ステップ目か) */
  int width, height;
  double x0, y0, x1, y1;
  double s0, t0, s1, t1;
  double distance;
  double vehicle_x = 0.;
  double vehicle_y = 0.;
  int steps_0, steps_1;
  double x_standard = 1.0;
  double y_standard = -1.0;
  double theta1;
//  int last_path;


  /* ファイルの読み込み */
  crossing_number = map_read("map2.dat");
  printf("loaded %d crossings\n",crossing_number);
  for(i=0;i<crossing_number;i++)
    cross[i].distance=0;    /* 適当に初期化しておきます for print_cross */

  /* 目的地の取得 */
  printf("目的地を決定します。");
  goal=search_cross(crossing_number);
  
  if(goal<0)
    return 1;    /* 目的地決定失敗 */

  printf("%d\n", goal);
  dijkstra(crossing_number,goal);
  print_cross_list(crossing_number);

  printf("現在地を決定します。");
  start=search_cross(crossing_number);
  if(start<0)
    return 1;

  if(pickup_path(crossing_number, start, goal, path, 0)<0)
    return 1;

  printf("経路確定しました。\n");
  i = 0;
  while(path[i]>=0)
    {
      printf("%2d %5.1lf %s\n",
	     i+1, cross[path[i]].distance, cross[path[i]].jname);
      i++;
    }


    printf("地図の東西南北に従う:0　進行方向を常に上向きにする:1 \n");			// ２パターンの表示を選択できるようにした
    printf("表示方法を選んでください:");
    scanf("%d", &d);									// 0 or 1で表示方法が変わる
    if(d == 0) d = -1;									// Cボタンのみで表示を変換できるように工夫

    a = 1;										// 物体の移動スピードの初期設定
	                                                           

    printf("\n");
    printf("拡大：L　縮小：S　縮図を戻す：F　表示変更：C　\n");
    printf("矢印キーで視点変更　　視点変更を戻す：R \n");
    printf("加速：A 一時停止：W \n");
    printf("\n");


       glfwInit();
    glfwOpenWindow(640, 640, 0, 0, 0, 0, 0, 0, GLFW_WINDOW);


    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();             /* それ以外の座標変換は行わない */

    /* 文字列描画のためのフォントの読み込みと設定 */
    font = ftglCreateExtrudeFont(FONT_FILENAME);
    if (font == NULL) {
        perror(FONT_FILENAME);
        fprintf(stderr, "could not load font\n");
        exit(1);
    }
//    ftglSetFontFaceSize(font, 14, 14);							// フォントサイズを文字が被らないように工夫
    ftglSetFontDepth(font, 0.01);
    ftglSetFontOutset(font, 0, 0.1);
    ftglSetFontCharMap(font, ft_encoding_unicode);

    /* マップファイルの読み込み */
    crossing_number = map_read("map2.dat");
    if (crossing_number < 0) {
        fprintf(stderr, "couldn't read map file\n");
        exit(1);
    }


    while (1) {

        /* Esc が押されるかウィンドウが閉じられたらおしまい */
        if (glfwGetKey(GLFW_KEY_ESC) || !glfwGetWindowParam(GLFW_OPENED))
            break;


        /* (ORIGIN_X, ORIGIN_Y) を中心に、REAL_SIZE_X * REAL_SIZE_Y の範囲の
         空間をビューポートに投影する */
	// SとLでサイズ変更できるようにWhile文の中に記述するよう工夫
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(ORIGIN_X + REAL_SIZE_X * -0.15, ORIGIN_X + REAL_SIZE_X * 0.15,
                    ORIGIN_Y + REAL_SIZE_Y * -0.15, ORIGIN_Y + REAL_SIZE_Y * 0.15,
                    -0.15, 0.15);


        glfwGetWindowSize(&width, &height); /* 現在のウィンドウサイズを取得する */
        glViewport(0, 0, width, height); /* ウィンドウ全面をビューポートにする */

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClear(GL_COLOR_BUFFER_BIT); /* バックバッファを黒で塗り潰す */

	glfwSetKeyCallback(KeyBoard);//拡大縮小、加速減速の関数を呼び出す

	// 矢印キーで視点変更できるよう工夫	
	glTranslated(VIEW_POINT_X, VIEW_POINT_Y, 0.15);


        /* 移動体を進めて座標を計算する */
        if (path[vehicle_pathIterator + 0] != -1 &&
                path[vehicle_pathIterator + 1] != -1)
	  {
            /* まだゴールに達していないので、移動体の位置を進める */


	    x0 = cross[path[vehicle_pathIterator + 0]].pos.x;
            y0 = cross[path[vehicle_pathIterator + 0]].pos.y;
            x1 = cross[path[vehicle_pathIterator + 1]].pos.x;
            y1 = cross[path[vehicle_pathIterator + 1]].pos.y;



          if(d == -1)
	    {			// d=-1なら矢印の先端を進行方向へと動かす、地図の東西南北を変更しない

	    distance = hypot(x1 - x0, y1 - y0);
            steps_0 = (int)(distance / 0.1);

	    

	    theta1 = atan2(y1-y0, x1-x0);

            /* 道路上を進んで、座標を更新 */
	    if(w == 1) vehicle_stepOnEdge++;										// Wが押されれば、一時停止　(eles省略、以下同様)

            vehicle_x = x0 + (x1 - x0) / steps_0 * vehicle_stepOnEdge;
            vehicle_y = y0 + (y1 - y0) / steps_0 * vehicle_stepOnEdge;

	    // 表示方法を変える
	    if(REAL_SIZE_X == 14.0) {											// 拡大縮小されなければ、ルートを表示する
	    map_motion_0(crossing_number, x_standard + 0.8, y_standard, vehicle_x, vehicle_y, path);			
	    hyouji(x_standard - 2.1 + delta_x, y_standard + 1.5 + delta_y, path, vehicle_pathIterator + 0);		// ルート表示がカーソルキーで移動しないようにdeltaを用いて平行移動させている
	    glColor3d(1.0, 0.1, 0.1);
	    draw_triangle_0(x_standard + 0.8, y_standard, theta1, MARKER_RADIUS);					
	    } else {													// 拡大されれば、画面中央に物体を表示し、ルート表示を消す
	    map_motion_0(crossing_number, x_standard, y_standard, vehicle_x, vehicle_y, path);				
	    glColor3d(1.0, 0.1, 0.1);											
	    draw_triangle_0(x_standard, y_standard, theta1, MARKER_RADIUS);						
	    }


	    if (vehicle_stepOnEdge >= steps_0) {
                /* 交差点に達したので次の道路へ入る */
                vehicle_pathIterator++;
                vehicle_stepOnEdge = 0;
            }

	  } else if(d == 1) {		// d=1なら地図を回転し、進行方向を常に上向きにする

	    map_change(crossing_number, x0, y0, x1, y1);

	    s0 = cross[path[vehicle_pathIterator + 0]].pos_c.x;
            t0 = cross[path[vehicle_pathIterator + 0]].pos_c.y;
            s1 = cross[path[vehicle_pathIterator + 1]].pos_c.x;
            t1 = cross[path[vehicle_pathIterator + 1]].pos_c.y;


            distance = hypot(s1 - s0, t1 - t0);
            steps_1 = (int)(distance / 0.1);

            /* 道路上を進んで、座標を更新 */
	    if(w == 1) vehicle_stepOnEdge++;										// Wが押されれば、一時停止
               
	    vehicle_x = s0 + (s1 - s0) / steps_1 * vehicle_stepOnEdge;
            vehicle_y = t0 + (t1 - t0) / steps_1 * vehicle_stepOnEdge;

	    // 表示方法を変える
	    if(REAL_SIZE_X == 14.0) {											// 拡大縮小されなければ、ルートを表示する
	    map_motion_1(crossing_number, x_standard + 0.8, y_standard, vehicle_x, vehicle_y, x0, y0, x1, y1, path);	
	    hyouji(x_standard - 2.1 + delta_x, y_standard + 1.5 + delta_y, path, vehicle_pathIterator + 0);		// ルート表示が、カーソルキーで移動しないようdeltaを用いて平行移動させている
	    glColor3d(1.0, 0.1, 0.1);											
	    draw_triangle_1(x_standard + 0.8, y_standard, MARKER_RADIUS);						
	    } else {													//　拡大縮小されれば、画面中央に物体を表示し、ルート表示を消す
	    map_motion_1(crossing_number, x_standard , y_standard, vehicle_x, vehicle_y, x0, y0, x1, y1, path);		
	    glColor3d(1.0, 0.1, 0.1);											
	    draw_triangle_1(x_standard, y_standard, MARKER_RADIUS);							
	    }

	    if (vehicle_stepOnEdge >= steps_1) {
                /* 交差点に達したので次の道路へ入る */
                vehicle_pathIterator++;
                vehicle_stepOnEdge = 0;
            }

	 }

        } else {

	  if(d == -1) {
	     if(REAL_SIZE_X == 14.0) {
	     map_motion_0(crossing_number, x_standard + 0.8, y_standard, x1, y1, path);					// 到着後に拡大縮小されなければ、ルートは表示されたまま
	     hyouji(x_standard - 2.1 + delta_x, y_standard + 1.5 + delta_y, path, vehicle_pathIterator + 0);		
	     /* 移動体を表示 */
 	     glColor3d(1.0, 1.0, 1.0);							// 到着後に白に変わる
	     draw_triangle_0(x_standard + 0.8, y_standard, theta1, MARKER_RADIUS);
	     } else {													// 到着後拡大縮小された場合は、画面中央で行い、ルートは表示しない
	    map_motion_0(crossing_number, x_standard, y_standard, x1, y1, path);					
	    glColor3d(1.0, 1.0, 1.0);
	    draw_triangle_0(x_standard, y_standard, theta1, MARKER_RADIUS);
	    }
	  } else if(d == 1) {
	     if(REAL_SIZE_X == 14.0) {
	     map_motion_1(crossing_number, x_standard + 0.8, y_standard, s1, t1, x0, y0, x1, y1, path);			// 到着後に拡大縮小されなければ、ルートは表示されたまま
	     hyouji(x_standard - 2.1 + delta_x, y_standard + 1.5 + delta_y, path, vehicle_pathIterator + 0);		
	     /* 移動体を表示 */
             glColor3d(1.0, 1.0, 1.0);							// 到着後に白に変わる
	     draw_triangle_1(x_standard + 0.8, y_standard, MARKER_RADIUS);
	     } else {													// 到着後拡大縮小された場合は、画面中央で行い、ルートは表示しない
	    map_motion_1(crossing_number, x_standard , y_standard, s1, t1, x0, y0, x1, y1, path);
	    glColor3d(1.0, 1.0, 1.0);
	    draw_triangle_1(x_standard, y_standard, MARKER_RADIUS);
	    }
	  }
	}


        glfwSwapBuffers(); /* フロントバッファとバックバッファを入れ替える */

        if(a == 1) usleep(500 * 1000); 							// 500ミリ秒くらい待つ 
        else if(a == -1) usleep(250 * 1000);						// 200ミリ秒くらい待つ
    }

    glfwTerminate();



  
  return 0;
}

  



