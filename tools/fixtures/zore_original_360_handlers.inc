/* ZORE original handlers; frame timeout removed and tested in ISR wrapper. */
void Element_Handle_Left_Rings()
{
    int num = 0;
		int  flag_Xsite_1=0;
    int flag_Ysite_1=0;
    long Slope_Rings_Int=0; 
	
		for (Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')
            num++;
        if(    ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
            && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
            break;
    }

    if (ImageFlag.image_element_rings_flag == 1 && num>12)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<LEFT_RING5TO6)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Right_Line>11)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Right_Line<5)
    {
        ImageFlag.image_element_rings_flag = 7;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for ( Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if (       ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 2].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite + 1].RightBorder
                    && ImageDeal[Ysite].RightBorder <= ImageDeal[Ysite - 1].RightBorder


               )
            {
                Point_Xsite = ImageDeal[Ysite].RightBorder;
                Point_Ysite = Ysite;
                break;
            }
        }
        if (Point_Ysite > 28)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (z360_Straight_Judge(2, ImageStatus.OFFLine+10, 45) < 512
             &&ImageStatus.Right_Line < 9
             && ImageStatus.OFFLine < 21)    
           {
               ImageFlag.image_element_rings_flag = 9;
               Element_Stop_Record_Completion(STOP_ELEMENT_RING);
           }
    }

    if (ImageFlag.image_element_rings_flag == 9)
    {
        int num=0;
        for (Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsLeftFind == 'W' )
                num++;
        }
        if(num < 8)
        {
            ImageStatus.Road_type = 0;   
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
        }
    }

    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for ( Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite]+2;
        }
    }

    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6
        )
    {
        flag_Xsite_1=0;
        flag_Ysite_1=0;
        Slope_Rings_Int=0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(  z360_px(Ysite,Xsite) == 1 && z360_px(Ysite,Xsite + 1) == 0)
                 {
                   flag_Ysite_1 = Ysite;
                   flag_Xsite_1 = Xsite;
                   
                   Slope_Rings_Int = ((long)(79 - flag_Xsite_1) * 256) / (59 - flag_Ysite_1);
                   break;
                 }
            }
            if(flag_Ysite_1 != 0)
            {
                break;
            }
        }
        if(flag_Ysite_1 == 0)
        {
            for(Ysite=ImageStatus.OFFLine;Ysite<30;Ysite++)
            {
                if(ImageDeal[Ysite].IsLeftFind=='T'&&ImageDeal[Ysite+1].IsLeftFind=='T'&&ImageDeal[Ysite+2].IsLeftFind=='W'
                    &&abs(ImageDeal[Ysite].LeftBorder-ImageDeal[Ysite+2].LeftBorder)>7
                  )
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=ImageDeal[flag_Ysite_1].LeftBorder;
                    ImageStatus.OFFLine=Ysite;
                    
                    Slope_Rings_Int = ((long)(79 - flag_Xsite_1) * 256) / (59 - flag_Ysite_1);
                    break;
                }
            }
        }

        if(flag_Ysite_1 != 0)
        {
            for(Ysite=flag_Ysite_1;Ysite<60;Ysite++)
            {
                
                ImageDeal[Ysite].RightBorder = flag_Xsite_1 + (int)((Slope_Rings_Int * (Ysite - flag_Ysite_1)) >> 8);
                ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                if(ImageDeal[Ysite].Center<4)
                    ImageDeal[Ysite].Center = 4;
            }
            ImageDeal[flag_Ysite_1].RightBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) 
            {
                for(Xsite=ImageDeal[Ysite+1].RightBorder-18;Xsite<ImageDeal[Ysite+1].RightBorder+2;Xsite++)
                {
                    if(z360_px(Ysite,Xsite)==1 && z360_px(Ysite,Xsite+1)==0)
                    {
                        ImageDeal[Ysite].RightBorder=Xsite;
                        ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder)/2);
                        if(ImageDeal[Ysite].Center<4)
                            ImageDeal[Ysite].Center = 4;
                        ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                        break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 &&ImageDeal[Ysite].RightBorder< ImageDeal[Ysite+2].RightBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }

    if (ImageFlag.image_element_rings_flag == 7)
    {
    }

    if (ImageFlag.image_element_rings_flag == 8 && ImageFlag.ring_big_small == 1)    
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for (Ysite = 40; Ysite > 5; Ysite--)
        {
            if (z360_px(Ysite,25) == 1 && z360_px(Ysite-1,25) == 0)
            {
                Repair_Point_Xsite = 25;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  
                break;
            }
        }
        for ( Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         
        {
            ImageDeal[Ysite].RightBorder = (ImageDeal[58].RightBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].RightBorder;
            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2);
        }
    }

    if (ImageFlag.image_element_rings_flag == 9 || ImageFlag.image_element_rings_flag == 10)
    {
        for ( Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Bend_Wide[Ysite];
        }
    }
}

void Element_Handle_Right_Rings()
{
    int num =0 ;
	int flag_Xsite_1;
    int flag_Ysite_1;
    long Slope_Right_Rings_Int; 
	
	for (Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if(    ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )
            break;
    }

    if (ImageFlag.image_element_rings_flag == 1 && num>12)
    {
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num<RIGHT_RING5TO6)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.Left_Line>10)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.Left_Line<6)
    {
        ImageFlag.image_element_rings_flag = 7;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (Ysite = 45; Ysite > ImageStatus.OFFLine+3; Ysite--)
        {
            if (    ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 2].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 1].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite + 3].LeftBorder
                 && ImageDeal[Ysite].LeftBorder >= ImageDeal[Ysite - 3].LeftBorder
                )

            {
                        Point_Xsite = ImageDeal[Ysite].LeftBorder;
                        Point_Ysite = Ysite;
                        break;
            }
        }
        if (Point_Ysite > 28)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }
    if (ImageFlag.image_element_rings_flag == 8)
    {
         if (   z360_Straight_Judge(1, ImageStatus.OFFLine+10, 45) < 768
             && ImageStatus.Left_Line < 11
             && ImageStatus.OFFLine < 22)    
            {
                ImageFlag.image_element_rings_flag = 9;
                Element_Stop_Record_Completion(STOP_ELEMENT_RING);
            }

    }
    if(ImageFlag.image_element_rings_flag == 9 )
    {
         num=0;
        for (Ysite = 40; Ysite > 10; Ysite--)
        {
            if(ImageDeal[Ysite].IsRightFind == 'W' )
            {
                num++;
            }
        }
        if(num < 5)
        {
            ImageStatus.Road_type = 0;   
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            ImageFlag.ring_big_small = 0;
            ImageStatus.Road_type = Normol;
        }
    }

    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
        }
    }

    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6
       )
    {
        flag_Xsite_1=0;
        flag_Ysite_1=0;
        Slope_Right_Rings_Int = 0;
        for(Ysite=55;Ysite>ImageStatus.OFFLine;Ysite--)
        {
            for(Xsite=ImageDeal[Ysite].LeftBorder + 1;Xsite<ImageDeal[Ysite].RightBorder - 1;Xsite++)
            {
                if(z360_px(Ysite,Xsite)==1 && z360_px(Ysite,Xsite+1)==0)
                {
                    flag_Ysite_1=Ysite;
                    flag_Xsite_1=Xsite;
                    
                    Slope_Right_Rings_Int = ((long)(0 - flag_Xsite_1) * 256) / (59 - flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
              break;
            }
        }
        if(flag_Ysite_1==0)
        {
        for(Ysite=ImageStatus.OFFLine+5;Ysite<30;Ysite++)
        {
         if(ImageDeal[Ysite].IsRightFind=='T'&&ImageDeal[Ysite+1].IsRightFind=='T'&&ImageDeal[Ysite+2].IsRightFind=='W'
              &&abs(ImageDeal[Ysite].RightBorder-ImageDeal[Ysite+2].RightBorder)>5
         )
         {
             flag_Ysite_1=Ysite;
             flag_Xsite_1=ImageDeal[flag_Ysite_1].RightBorder;
             ImageStatus.OFFLine=Ysite;
             
             Slope_Right_Rings_Int = ((long)(0 - flag_Xsite_1) * 256) / (59 - flag_Ysite_1);
             break;
         }

        }
        }

        if(flag_Ysite_1!=0)
        {
            for(Ysite=flag_Ysite_1;Ysite<58;Ysite++)
            {
                
                ImageDeal[Ysite].LeftBorder = flag_Xsite_1 + (int)((Slope_Right_Rings_Int * (Ysite - flag_Ysite_1)) >> 8);
                ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;
                if(ImageDeal[Ysite].Center>79)
                    ImageDeal[Ysite].Center=79;
            }
            ImageDeal[flag_Ysite_1].LeftBorder=flag_Xsite_1;
            for(Ysite=flag_Ysite_1-1;Ysite>10;Ysite--) 
            {
                for(Xsite=ImageDeal[Ysite+1].LeftBorder+8;Xsite>ImageDeal[Ysite+1].LeftBorder-4;Xsite--)
                {
                    if(z360_px(Ysite,Xsite)==1 && z360_px(Ysite,Xsite-1)==0)
                    {
                     ImageDeal[Ysite].LeftBorder=Xsite;
                     ImageDeal[Ysite].Wide=ImageDeal[Ysite].RightBorder-ImageDeal[Ysite].LeftBorder;
                     ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder+ImageDeal[Ysite].RightBorder)/2;
                     if(ImageDeal[Ysite].Center>79)
                         ImageDeal[Ysite].Center=79;
                     if(ImageDeal[Ysite].Center<5)
                         ImageDeal[Ysite].Center=5;
                     break;
                    }
                }
                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder>  ImageDeal[Ysite+2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite+2;
                    break;
                }
            }
        }
    }

    if (ImageFlag.image_element_rings_flag == 7)
    {
    }

    if(ImageFlag.image_element_rings_flag == 8)  
    {
        Repair_Point_Xsite = 20;
        Repair_Point_Ysite = 0;
        for ( Ysite = 40; Ysite > 8; Ysite--)
        {
            if (z360_px(Ysite,50) == 1 && z360_px(Ysite-1,50) == 0)
            {
                Repair_Point_Xsite = 50;
                Repair_Point_Ysite = Ysite-1;
                ImageStatus.OFFLine = Ysite + 1;  
                break;
            }
        }
        for (Ysite = 57; Ysite > Repair_Point_Ysite-3; Ysite--)         
        {
            ImageDeal[Ysite].LeftBorder = (ImageDeal[58].LeftBorder - Repair_Point_Xsite) * (Ysite - 58) / (58 - Repair_Point_Ysite)  + ImageDeal[58].LeftBorder;
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
        }
    }

    if (ImageFlag.image_element_rings_flag == 9)
    {
        for ( Ysite = 59; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Bend_Wide[Ysite];
        }
    }
}