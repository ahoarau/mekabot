#Copyright  2010, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted. 


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import Numeric as nu
#How well does x match y. Return Pct
def measure_accuracy(x,y):
    #yy=nu.maximum(abs(nu.array(y)),.00001) #avoid NAN
    #xx=nu.maximum(abs(nu.array(x)),.00001) #avoid NAN
    return 0#100.0*sum((xx-yy)/yy)/len(yy)
    

def measure_avg_error(x,y):
    xx=nu.array(x)
    yy=nu.array(y)
    return sum(abs(xx-yy))/len(xx)

def measure_max_error(x,y):
    xx=nu.array(x)
    yy=nu.array(y)
    return max(abs(xx-yy))

def measure_stats(x,y):
    stats={'accuracy_pct':measure_accuracy(x,y),
           'avg_error':measure_avg_error(x,y),
           'max_error': measure_max_error(x,y)}
    
    
    
    