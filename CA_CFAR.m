function CFAR_position = CA_CFAR(mtd,guardCell,referenceCell)
    abs_mtd = abs(mtd);
    row = size(abs_mtd,1);
    column = size(abs_mtd,2);
    
    rowStartIndex = 1+guardCell+referenceCell;
    rowEndIndex = row-guardCell-referenceCell;
    
    columnStartIndex = 1+guardCell+referenceCell;
    columnEndIndex = column-guardCell-referenceCell; 
    
    CFAR_num = 0;
    
    for ii=rowStartIndex:rowEndIndex
        for jj=columnStartIndex:columnEndIndex
            sum = 0;
            for kk=1:referenceCell
                dataBlock = abs_mtd(ii-guardCell-kk:ii+guardCell+kk,jj-guardCell-kk:jj+guardCell+kk);
                sum = sum + HUI_sum(dataBlock,1+guardCell+kk,1+guardCell+kk);
            end
            num = 2*referenceCell*(2+4*guardCell+2*referenceCell);
            meanData = sum/num;
            if abs_mtd(ii,jj) > meanData*10
                CFAR_num = CFAR_num + 1;
                CFAR_position(CFAR_num,:) = [ii,jj];
            end
        end
    end
end


function result = HUI_sum(dataBlock,length,width)
    result = 0;
    for ii=1:length
        result = result + dataBlock(1,ii) + dataBlock(width,ii);
    end
    for ii=2:width-1
        result = result + dataBlock(ii,1) + dataBlock(length,ii);
    end
end

