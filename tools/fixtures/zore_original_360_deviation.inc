#define Straight 6
#define Ramp 3
#define Obstacle 7
static void original_GetDet(void)
{
  long DetTemp_Int = 0; 
  long UnitAll_Int = 0; 
  int TowPoint = 0;

int calc_near_row = ImageStatus.NearPoint;
	
  if (calc_near_row <= ImageStatus.OFFLine) {
      calc_near_row = ImageStatus.OFFLine + 1; 
  }
  
  
  if (calc_near_row > 59) {
      calc_near_row = 59;
  }
	if (ImageStatus.Road_type == RightCirque || ImageStatus.Road_type == LeftCirque)
{
    switch (ImageFlag.image_element_rings_flag)
    {
        case 1:  TowPoint = 21; break;
        case 2:  TowPoint = 21; break;
        case 3:  TowPoint = 24; break;
        case 4:  TowPoint = 24; break;
        case 5:  TowPoint = 21; break;
        case 6:  TowPoint = 26; break;
        case 7:  TowPoint = 20; break;
        case 8:  TowPoint = 20; break;
        case 9:  TowPoint = 24; break;
        default: TowPoint = 22; break;
    }
}
                                                                            
  else if (ImageStatus.Road_type == Straight)     TowPoint = SystemData.straighet_towpoint;
  else if(ImageStatus.Road_type ==Cross_ture)     TowPoint=23;
 else if(ImageStatus.Road_type ==Cross_mid)     TowPoint=23;
  else if(ImageStatus.Road_type ==Ramp)           TowPoint=32;
  else if(ImageStatus.Road_type ==Obstacle)       TowPoint=30;

  else
      TowPoint = ImageStatus.TowPoint;                                              

  if (TowPoint < ImageStatus.OFFLine) TowPoint = ImageStatus.OFFLine + 1;
  if (TowPoint >= 49) TowPoint = 49;

  
  
  
  if ((TowPoint - 5) >= ImageStatus.OFFLine) {                                          
      
      
      
      for (Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) {
          DetTemp_Int += Weight_Far[TowPoint - Ysite - 1] * ImageDeal[Ysite].Center;
          
      }
      
      
      for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) {
          DetTemp_Int += Weight_Near[-TowPoint + Ysite - 1] * ImageDeal[Ysite].Center;
      }
      
      
      ImageStatus.Det_True = (ImageDeal[TowPoint].Center * 96 + DetTemp_Int) >> 8;

  } else if (TowPoint > ImageStatus.OFFLine) {
      
      for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) {
          DetTemp_Int += Weight_Far[TowPoint - Ysite - 1] * ImageDeal[Ysite].Center;
          UnitAll_Int += Weight_Far[TowPoint - Ysite - 1];
      }
      for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint; Ysite--) {
          DetTemp_Int += Weight_Near[-TowPoint + Ysite - 1] * ImageDeal[Ysite].Center;
          UnitAll_Int += Weight_Near[-TowPoint + Ysite - 1];
      }
      ImageStatus.Det_True = (ImageDeal[TowPoint].Center * 96 + DetTemp_Int) / (UnitAll_Int + 96);
      
  } else if (ImageStatus.OFFLine < 49) {
      
      for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine; Ysite--) {
          DetTemp_Int += Weight_Near[-TowPoint + Ysite - 1] * ImageDeal[Ysite].Center;
          UnitAll_Int += Weight_Near[-TowPoint + Ysite - 1];
      }
      ImageStatus.Det_True = (ImageDeal[ImageStatus.OFFLine].Center * 96 + DetTemp_Int) / (UnitAll_Int + 96);
  }

  ImageStatus.TowPoint_True = TowPoint;  

}