/* ZORE reference admission functions, extracted without predicate edits.
 * https://github.com/ZORE-dlfd/21st-smart-car-yan-guo-liu-hen-8of10-2.6mps
 * Unit-test oracle only; does not compile into firmware. */
void Element_Judgment_Left_Rings()
{
	 int ring_ysite;
		int ysite;
	int narrow_row_count = 0;
	for (ysite = 57; ysite >= 54; ysite--)
    {
        // 如果脚下的某一行不仅没丢边，而且宽度还极其狭窄，说明是被路障挤压�?
        if (ImageDeal[ysite].IsLeftFind == 'T' && ImageDeal[ysite].IsRightFind == 'T' && ImageDeal[ysite].Wide < safe_wide_thresh)
        {
            narrow_row_count++;
        }
    }
		
		if (narrow_row_count >= 3) 
    {
        return; 
    }
    if (
              ImageStatus.Right_Line >2 
            ||ImageStatus.Left_Line < LEFT_RING_LOSS 
            ||ImageStatus.OFFLine > 23
            || ImageStatus.WhiteLine>2
//		        || ImageDeal[54].IsLeftFind == 'W'
            //|| ImageDeal[55].IsLeftFind == 'W'
            //|| ImageDeal[56].IsLeftFind == 'W'
            //|| ImageDeal[57].IsLeftFind == 'W'
            //|| ImageDeal[58].IsLeftFind == 'W'
	        || Global_Straight_R> 200
        )
    { return;}
    
    ring_ysite = 25;
    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;
    for (Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for ( Ysite = Left_RingsFlag_Point1_Ysite; Ysite > ImageStatus.OFFLine; Ysite--)
    {
        if (   ImageDeal[Ysite + 6].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 5].LeftBorder < ImageDeal[Ysite+3].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 2].LeftBorder
            && ImageDeal[Ysite + 3].LeftBorder > ImageDeal[Ysite + 1].LeftBorder
            )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 0 )
    {
        if(ImageStatus.Left_Line > 20)
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite > Left_RingsFlag_Point1_Ysite+1 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag ==0)
    {
        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;
        ImageStatus.Road_type = LeftCirque;
			
			 gpio_set_level(BEEP_PIN, 1);
    }
    Ring_Help_Flag = 0;
}

void Element_Judgment_Right_Rings()
{
	int ring_ysite;
	int ysite;
	int narrow_row_count = 0;
	for (ysite = 59; ysite >= 54; ysite--)
    {
        // 如果脚下的某一行不仅没丢边，而且宽度还极其狭窄，说明是被路障挤压�?
        if (ImageDeal[ysite].IsLeftFind == 'T' && ImageDeal[ysite].IsRightFind == 'T' && ImageDeal[ysite].Wide < safe_wide_thresh)
        {
            narrow_row_count++;
        }
    }
		
		if (narrow_row_count >= 2) 
    {
        return; 
    }
    if (   ImageStatus.Left_Line > 1
        || ImageStatus.Right_Line < RIGHT_RING_LOSS 
        || ImageStatus.OFFLine > 23
        || Global_Straight_L > 90
        || ImageStatus.WhiteLine>2
        //|| ImageDeal[52].IsRightFind == 'W'
        //|| ImageDeal[53].IsRightFind == 'W'
        //|| ImageDeal[54].IsRightFind == 'W'
        //|| ImageDeal[55].IsRightFind == 'W'
        //|| ImageDeal[56].IsRightFind == 'W'
        //|| ImageDeal[57].IsRightFind == 'W'
        //|| ImageDeal[58].IsRightFind == 'W'
        )
        { return;}
        
    ring_ysite = 25;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    for ( Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (Ysite = 58; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }
    for( Ysite = Right_RingsFlag_Point1_Ysite; Ysite > 10; Ysite--)
    {
        if (   ImageDeal[Ysite + 6].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 5].RightBorder > ImageDeal[Ysite + 3].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 2].RightBorder
            && ImageDeal[Ysite + 3].RightBorder < ImageDeal[Ysite + 1].RightBorder
           )
        {
            Ring_Help_Flag = 1;
            break;
        }
    }
    if(Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.Right_Line>19)
            Ring_Help_Flag = 1;
    }
    if (Right_RingsFlag_Point2_Ysite > Right_RingsFlag_Point1_Ysite+3 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0)
    {
        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;
        ImageFlag.ring_big_small=1;     
        ImageStatus.Road_type = RightCirque;
			 gpio_set_level(BEEP_PIN, 1);
    }
    Ring_Help_Flag = 0;
}

