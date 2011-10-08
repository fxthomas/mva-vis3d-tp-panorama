// Imagine++ project
// Project:  Panorama
// Author:   Pascal Monasse
// Date:     2011/09/26

#include <Imagine/Graphics.h>
#include <Imagine/Images.h>
#include <Imagine/LinAlg.h>
#include <vector>
#include <sstream>
using namespace Imagine;
using namespace std;

// Record clicks in two images, until right button click
void getClicks(Window w1, Window w2, vector<IntPoint2>& pts1, vector<IntPoint2>& pts2) {
  int button = 0, subWin = 0;
  Window win;
  IntPoint2 point;

  while (true) {
    // Capture click (and exit loop if right click)
    setActiveWindow (w1);
    button = anyGetMouse (point, win, subWin);
    if (button == 3) break;

    cout << "Captured point for window 1 : (" << point << ")" << endl;
    if (win == w1) pts1.push_back (point);
    else continue;

    // Capture click (and exit loop if right click)
    setActiveWindow (w2);
    button = anyGetMouse (point, win, subWin);
    if (button == 3) { pts1.pop_back(); break; }

    cout << "Captured point for window 2 : (" << point << ")" << endl;
    if (win == w2) pts2.push_back (point);
    else pts1.pop_back();
  }
}

// Return homography compatible with point matches
Matrix<float> getHomography(const vector<IntPoint2>& pts1,
    const vector<IntPoint2>& pts2) {
  size_t n = min(pts1.size(), pts2.size());
  Matrix<float> A(2*n,8);
  Vector<float> B(2*n);

  for (int i = 0; i < n; i++) {
    IntPoint2 p1 = pts1[i];
    IntPoint2 p2 = pts2[i];

    B[2*i] = p2.x();
    B[2*i+1] = p2.y();

    A(2*i, 0) = p1.x();
    A(2*i, 1) = p1.y();
    A(2*i, 2) = 1;
    A(2*i, 3) = 0;
    A(2*i, 4) = 0;
    A(2*i, 5) = 0;
    A(2*i, 6) = -p2.x()*p1.x();
    A(2*i, 7) = -p2.x()*p1.y();
    A(2*i+1, 0) = 0;
    A(2*i+1, 1) = 0;
    A(2*i+1, 2) = 0;
    A(2*i+1, 3) = p1.x();
    A(2*i+1, 4) = p1.y();
    A(2*i+1, 5) = 1;
    A(2*i+1, 6) = -p2.y()*p1.x();
    A(2*i+1, 7) = -p2.y()*p1.y();

    i++;
  }

  B = linSolve(A, B);
  Matrix<float> H(3, 3);
  H(0,0)=B[0]; H(0,1)=B[1]; H(0,2)=B[2];
  H(1,0)=B[3]; H(1,1)=B[4]; H(1,2)=B[5];
  H(2,0)=B[6]; H(2,1)=B[7]; H(2,2)=1;
  cout << H << endl;

  // Sanity check
  for(size_t i=0; i<n; i++) {
    float v1[]={(float)pts1[i].x(), (float)pts1[i].y(), 1.0f};
    float v2[]={(float)pts2[i].x(), (float)pts2[i].y(), 1.0f};
    Vector<float> x1(v1,3);
    Vector<float> x2(v2,3);
    x1 = H*x1;
    cout << x1[1]*x2[2]-x1[2]*x2[1] << ' '
      << x1[2]*x2[0]-x1[0]*x2[2] << ' '
      << x1[0]*x2[1]-x1[1]*x2[0] << endl;
  }
  return H;
}

// Grow rectangle of corners (x0,y0) and (x1,y1) to include (x,y)
void growTo(float& x0, float& y0, float& x1, float& y1, float x, float y) {
  if(x<x0) x0=x;
  if(x>x1) x1=x;
  if(y<y0) y0=y;
  if(y>y1) y1=y;    
}

// Panorama construction
void panorama(const Image<Color,2>& I1, const Image<Color,2>& I2,
    Matrix<float> H) {
  Vector<float> v(3);
  float x0=0, y0=0, x1=I2.width(), y1=I2.height();

  // Compute new bounding box
  v[0]=0; v[1]=0; v[2]=1;
  v=H*v; v/=v[2];
  growTo(x0, y0, x1, y1, v[0], v[1]);

  v[0]=I1.width(); v[1]=0; v[2]=1;
  v=H*v; v/=v[2];
  growTo(x0, y0, x1, y1, v[0], v[1]);

  v[0]=I1.width(); v[1]=I1.height(); v[2]=1;
  v=H*v; v/=v[2];
  growTo(x0, y0, x1, y1, v[0], v[1]);

  v[0]=0; v[1]=I1.height(); v[2]=1;
  v=H*v; v/=v[2];
  growTo(x0, y0, x1, y1, v[0], v[1]);

  cout << "x0 x1 y0 y1=" << x0 << ' ' << x1 << ' ' << y0 << ' ' << y1<<endl;

  // Create panorama
  Image<Color> I(int(x1-x0), int(y1-y0));
  setActiveWindow( openWindow(I.width(), I.height()) );
  I.fill(WHITE);

  for (int i = 0; i < I2.width(); i++) for (int j = 0; j < I2.height(); j++) I(i-int(x0),j-int(y0)) = I2(i,j);
  /*for (int i = 0; i < I1.width(); i++) for (int j = 0; j < I1.height(); j++) {
    v[0] = i; v[1] = j; v[2] = 1; v = H*v; v /= v[2];
    I(int(v[0]), int(v[1])) = I1(i,j);
  }*/

  display(I,0,0);
}

// Main function
int main(int argc, char** argv) {
  if(argc!=1 && argc!=3) {
    cout << "Usage: " << argv[0] << " [image1 image2]" <<endl;
    return 1;
  }
  const char* s1 = (argc==3)? argv[1]: "data/image0006.jpg";
  const char* s2 = (argc==3)? argv[2]: "data/image0007.jpg";

  // Load and display images
  Image<Color> I1, I2;
  if( ! load(I1, "data/image0006.jpg") ||
      ! load(I2, "data/image0007.jpg") ) {
    cerr<< "Impossible de charger les images" << endl;
    return 1;
  }
  Window w1 = openWindow(I1.width(), I1.height());
  display(I1,0,0);
  Window w2 = openWindow(I2.width(), I2.height());
  setActiveWindow(w2);
  display(I2,0,0);

  // Get user's clicks in images
  vector<IntPoint2> pts1, pts2;
  getClicks(w1, w2, pts1, pts2);

  vector<IntPoint2>::const_iterator it;
  cout << "pts1="<<endl;
  for(it=pts1.begin(); it != pts1.end(); it++)
    cout << *it << endl;
  cout << "pts2="<<endl;
  for(it=pts2.begin(); it != pts2.end(); it++)
    cout << *it << endl;

  // Compute homography
  Matrix<float> H = getHomography(pts1, pts2);

  // Apply homography
  panorama(I1, I2, H);

  endGraphics();
  return 0;
}
