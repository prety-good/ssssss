%{
int user_select,total=0;
%}
%%
dd[d|q]        {printf("请选糖：1、太妃糖  2、巧克力\n");
		scanf("%d",&user_select);
		if(user_select==1) printf("你选择了太妃糖\n");
		if(user_select==2) printf("你选择了巧克力\n");
		total++;
		}
dq		{printf("请选糖：1、太妃糖  2、巧克力\n");
		scanf("%d",&user_select);
		if(user_select==1) printf("你选择了太妃糖\n");
		if(user_select==2) printf("你选择了巧克力\n");
		total++;
		}
q               {printf("请选糖：1、太妃糖  2、巧克力\n");
                scanf("%d",&user_select);
                if(user_select==1) printf("你选择了太妃糖\n");
                if(user_select==2) printf("你选择了巧克力\n");
		total++;
		}
. 		{printf("投币非法\n");
		}
%%
main()
{
	printf("请投币\n");
	yylex();
	printf("共购买了%d块糖果\n",total);
}
int yywrap()
{return 1;}
