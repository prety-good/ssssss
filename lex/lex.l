 /*sun zhong wu*/
%{
	int usr_select,num;
%}

%%
	
nq|q|nnq|dq|ndq|ndd|nnnq|nnnd|dnq|dnd|nndn|nndd|nndq|ddn|ddd|ddq|ndnn|ndnd|ndnq|nnnnn|nnnnd|nnnnq|dnnn|dnnd|dnnq    {printf("请选择糖果：1、太妃糖 2、巧克力\n");
	scanf("%d",&usr_select);
	if(usr_select==1) printf("你选择了太妃糖！\n");
	if(usr_select==2) printf("你选择了巧克力！\n");
	num++;
	}
.      {printf("你投入了错误的硬币！输入非法！\n");
     	}

%%
main()
{
	system("PAUSE");
	printf("请投币：\nn:五美元  d:十美元  q:二十五美元\n");
	yylex();
	printf("你一共买了%d块糖\n",num);
	return 0;
}
int yywrap()
{return 1;}

