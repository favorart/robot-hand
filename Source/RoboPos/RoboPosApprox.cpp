#include "StdAfx.h"
#include "Robo.h"
#include "RoboMovesStore.h"
#include "RoboPosApprox.h"


using namespace RoboPos;


Point RoboPos::ApproxStore::predict(const Robo::Control &controls) const
{

   // tx = self.norm*rx
   // mk = np.sum(tx**2,axis=0,keepdims=True);
   // md = mk.transpose()+self.mk-2*np.matmul(tx.transpose(),self.tk)
   //
   // mt = md<=0
   // md[mt] = 1
   // md = md*(np.log(md)-self.t)
   // md[mt] = 0
   // k = tx.shape[1]
   // md = np.hstack((md,np.ones((k,1))))
   //
   // var = np.matmul(md,self.kof)
   // return var.transpose()

    Row x = convertRow(controls);
    return { y_function(x, q_l1), y_function(x, q_l2) };
}

bool RoboPos::ApproxStore::clarify(const Robo::Control &controls, Point hit)
{
    return false;
}

bool RoboPos::ApproxStore::learn()
{
    linSystQ(q_l1, 0);
    linSystQ(q_l2, 1);
    return false;
}

void RoboPos::ApproxStore::linSystQ(Row &q, bool i)
{
    std::vector<Row> a;
    // решить линейное уравнение...
}


RoboPos::ApproxStore::Row RoboPos::ApproxStore::convertRow(const Robo::Control &controls) const
{
    Row res(_max_controls_size);
    for (auto i = 0u; i < _max_controls_size; ++i)
    {
        for (auto &c : controls)
        {
        
        }
    }
    return res;
}

//class Mlearn:
//    def __init__(self,tk,rez,sm,norm=0):
//        if norm==0:
//            norm = (tk.max(axis=1,keepdims=True)-tk.min(axis=1,keepdims=True))*(tk.shape[0]**0.5)
//            g = norm==0
//            norm[g] = 1
//            self.norm = 1/norm
//        else:
//            self.norm = norm
//            
//        tx = self.norm*tk
//        self.tk = tx
//        self.t = 50
//        self.sm = sm
//        mk = np.sum(tx**2,axis=0,keepdims=True);
//        self.mk = mk
//        md = mk+mk.transpose()-2*np.matmul(tx.transpose(),tx)
//        
//        mt = md<=0
//        md[mt] = 1
//        md = md*(np.log(md)-self.t)
//        md[mt] = 0
//        k = tk.shape[1]
//        b = md.reshape((k*k,1))
//        b[np.arange(k)*(k+1)] = sm
//        md = np.vstack((md,np.ones((1,k))))
//        md = np.hstack((md,np.ones((k+1,1))))
//        md[k,k] = 0
//
//        ob = np.linalg.inv(md)
//        #self.kof = np.matmul(ob,np.vstack((rez.transpose(),np.zeros((1,rez.shape[0])))))
//        self.kof = np.linalg.solve(md,np.vstack((rez.transpose(),np.zeros((1,rez.shape[0])))))
//
//    def calculate(self,rx):
//        {
//            }

