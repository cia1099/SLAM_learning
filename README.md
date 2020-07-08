> 《SLAM視覺十四講》 ISBN：9789865501044 https://github.com/gaoxiang12/slambook2

<span id="contents"></span>
SLAM
====
<a href="contents">

Contents
 * [3D空間方體運動](#contents)
 * [李群和李代數](#ch4)

 ### 3. 3D空間剛體運動
 3.3.1 旋轉向量
 從旋轉向量到旋轉矩陣的轉換過程由**Rodrigues's Formula**：
 <div align=center>

<img src="http://latex.codecogs.com/gif.latex?\mathbf{R}=\cos\theta\mathbf{I}+(1-\cos\theta)\mathbf{nn^T}+\sin\theta\mathbf{n}^\wedge"/>
</div>

符號^是向量到screw-symmetric matrix的轉換符(外積cross操作變換)，對兩邊取trace可得：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\theta=\arccos\frac{tr(\mathbf{R})-1}{2}"/>
</div>

關於轉軸![](https://render.githubusercontent.com/render/math?math=\mathbf{n})，由旋轉軸上的向量在旋轉後不發生改變，說明：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\mathbf{Rn}=\mathbf{n}"/>
</div>

因此轉軸![](https://render.githubusercontent.com/render/math?math=\mathbf{n})是矩陣![](https://render.githubusercontent.com/render/math?math=\mathbf{R})特徵值1的對應特徵向量。

3.3.2 尤拉角
它等於ZYX軸的旋轉，可以使用[r,p,y]T這樣一個向量描述任意旋轉，但尤拉角的缺點是會有Gimbal lock的問題；在對Y軸旋轉±90時，就會讓X軸變成-Z軸，因而在對X軸旋轉等於對Z方向軸轉，所以遺失了第三次旋轉。
1. yaw，繞物體Z軸旋轉
2. pitch，繞旋轉之後的Y軸旋轉
3. roll，繞旋轉之後的X軸旋轉

3.4 四元數(Quaterion)
可以用四元數描述3D空間中任意一個旋轉，且這表示式不具奇異性，四元數是複數的3D版：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\mathbf{q}=q_0+q_1i+q_2j+q_3k"/>
</div>

其中i,j,k這三個虛數，滿足以下關係式：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\begin{cases}
i^2=j^2=k^2=-1\\
ij=k,ji=-k\\
jk=i,kj=-i\\
ki=j,ik=-j
\end{cases}">
</div>
在[p.3-24-3.25]有證明四元數到旋轉向量的轉換公式可得：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\begin{cases}
\theta=2arccosq_0\\
(n_x,n_y,n_z)=\frac{(q_1,q_2,q_3)}{sin\frac{\theta}{2}}
\end{cases}"/>
</div>

[Eigen對應MATLAB操作](https://igl.ethz.ch/projects/libigl/matlab-to-eigen.html)

<span id="ch4"></span>
### 4. 李群和李代數
<span style="background-color:yellow">群(Group)是一種集合加上一種運算的代數結構。</span>我們以旋轉矩陣為例，旋轉對加法是不封閉的![](http://latex.codecogs.com/gif.latex?\mathbf{R}_1+\mathbf{R}_2\notin{SO(3)})，而乘法是封閉的![](http://latex.codecogs.com/gif.latex?\mathbf{R}_1\mathbf{R}_2\in{SO(3)})，因此對於這種只有一個良好的運算集合，我們稱之為群。

李代數就是用來解exponetial的矩陣運算，使得指數的矩陣具有對數相加的近似對映由[Baker-Campbell-Hausdorff](https://en.wikipedia.org/wiki/Baker%E2%80%93Campbell%E2%80%93Hausdorff_formula)公式近似出：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\exp(\Delta\mathbf{\Phi}^\wedge)\exp(\mathbf{\Phi}^\wedge)=\exp[(\mathbf{\Phi}+\mathbf{J}^{-1}_l\Delta\mathbf{\Phi})^\wedge]"/>

or
<img src="http://latex.codecogs.com/gif.latex?\exp[(\mathbf{J}_l\Delta\mathbf{\Phi})^\wedge]\exp(\mathbf{\Phi}^\wedge)=\exp[(\mathbf{\Phi}+\Delta\mathbf{\Phi})^\wedge]=\exp(\mathbf{\Phi}^\wedge)\exp[(\mathbf{J}_r\Delta\mathbf{\Phi})^\wedge]"/>
</div>
其中近似雅可比矩陣：
<div align=center>

<img src="http://latex.codecogs.com/gif.latex?\mathbf{J}_l=\frac{\sin\theta}{\theta}\mathbf{I}+(1-\frac{\sin\theta}{\theta})\mathbf{aa^T}+\frac{1-\cos\theta}{\theta}\mathbf{a}^\wedge"/>


<img src="http://latex.codecogs.com/gif.latex?\mathbf{J}^{-1}_l=\frac{\theta}{2}\cot\frac{\theta}{2}\mathbf{I}+(1-\frac{\theta}{2}\cot\frac{\theta}{2})\mathbf{aa^T}+\frac{\theta}{2}\mathbf{a}^\wedge"/>

and
<img src="http://latex.codecogs.com/gif.latex?\mathbf{J}_r(\mathbf{\Phi})=\mathbf{J}_l(-\mathbf{\Phi})"/>
</div>

因為我們從微分方程解得旋轉矩陣有exponetial解[p.4-5-p.4.6]![](http://latex.codecogs.com/gif.latex?\mathbf{R}=\exp({\mathbf{\Phi}^\wedge})=\exp(\theta\mathbf{a}^\wedge)=\sum^\infty_{n=0}{\frac{1}{n!}(\theta\mathbf{a}^\wedge)^n})，該加總可推得**Rodrigues's Formula**；即我們將李代數so3中的任意元素![](http://latex.codecogs.com/gif.latex?\mathbf{\Phi}=\theta\mathbf{a})，<span style="background-color:yellow">可知![](http://latex.codecogs.com/gif.latex?\mathbf{\Phi})的大小就是旋轉角度，單位方向**a**是旋轉軸。</span>Jl和Jr差在左乘還是右乘的指數矩陣相乘順序在對數相加上的對映。
<img src="./img/ch4.png"/>
